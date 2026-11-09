import socket
from collections import deque
from threading import Thread, Lock, Event, Condition, current_thread
import time
from typing import Optional, List, Dict, Union, Any

TRIM_MAX = 1000
TRIM_MIN = -1000
MSP_HEADER_IN = "244d3c"

# Battery Constants (1S LiPo)
BATTERY_MAX_VOLTAGE = 4.2  # Fully charged
BATTERY_IDLE_VOLTAGE = 3.7  # Nominal/idle
BATTERY_CRITICAL_VOLTAGE = 3.0  # Critical low
BATTERY_DEFAULT_CAPACITY_MAH = 600  # Default battery capacity

# MSP Protocol Commands

MSP_FC_VERSION = 3
MSP_RAW_IMU = 102
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_SET_RAW_RC = 200
MSP_ACC_CALIBRATION = 205
MSP_MAG_CALIBRATION = 206
MSP_SET_MOTOR = 214
MSP_SET_ACC_TRIM = 239
MSP_ACC_TRIM = 240
MSP_EEPROM_WRITE = 250
MSP_SET_POS = 216
MSP_SET_COMMAND = 217
MSP_SET_1WIRE = 243
MSP_VARIO = 122  # Example MSP command for vario (check your drone's documentation for the correct value)
RETRY_COUNT = 3
FRAME_MAX_AGE = 0.5  # seconds: getters wait for a newer reply than this

# WS2812B RGB LED strip (needs MagisV2 firmware with MSP 225 / 226)
MSP_SET_RGB_LED = 225   # [index, r, g, b, brightness %], index 0xFF = all LEDs
MSP_RGB_LED_CTRL = 226  # [op, count]: op 1 = take the strip, op 0 = release
RGB_LED_ALL = 0xFF
RGB_MAX_LEDS = 64
LED_QUEUE_MAX = 64      # queued LED packets before set_led() waits for the drone

class Pluto:
    def __init__(self, ip='192.168.4.1', port=23):
        # RC channel values
        self.rcRoll = 1500
        self.rcPitch = 1500
        self.rcThrottle = 1500
        self.rcYaw = 1500
        self.rcAUX1 = 1500
        self.rcAUX2 = 1000
        self.rcAUX3 = 1500
        self.rcAUX4 = 1000
        self.command_type = 0
        self.droneRC = [1500, 1500, 1500, 1500, 1500, 1000, 1500, 1000]
        
        # Command types
        self.NONE_COMMAND = 0
        self.TAKE_OFF = 1
        self.LAND = 2
        
        # Connection state
        self.connected = False
        self.cam_connected = False
        self.TCP_IP = ip
        self.TCP_PORT = port
        
        # MSP Protocol Version (Pluto drones use Protocol V1)
        self.MSP_Protocol_Version = 1  # Default to V1 for Pluto drones
        
        # Enhanced battery telemetry (Protocol Version 1)
        self.vBatComp = 0      # Battery voltage compensated
        self.mAmpRaw = 0       # Current in mA
        self.mAhDrawn = 0      # mAh consumed
        self.mAhRemain = 0     # mAh remaining
        self.soc_Fused = 0     # State of charge (0-100%)
        self.auto_LandMode = 0 # Auto-land mode status
        
        # Basic battery telemetry (Other versions)
        self.bytevbat = 0      # Battery voltage (byte)
        self.pMeterSum = 0     # Power meter sum
        self.rssi = 0          # Signal strength
        self.amperage = 0      # Current draw
        
        # Thread safety
        self._lock = Lock()  # Protects shared state (RC values, command_type)
        self._send_lock = Lock()  # One packet at a time on the socket (writer thread + user calls)
        self._led_queue = deque()  # LED packets, sent by write_function one per cycle
        self._led_inflight = False  # write_function has popped an LED packet and not sent it yet
        self._stop_event = Event()  # Signals thread to stop
        self.thread = None
        self.client = None

        # Receiving (reader thread)
        self.reader_thread = None
        self._rx_buf = bytearray()     # received bytes not yet parsed into frames
        self._rx_text = bytearray()    # raw serial text waiting for a newline (dev mode)
        self._frames = {}              # MSP command -> (latest reply payload, time received)
        self._frames_cv = Condition()  # wakes getters waiting for a new reply
        self._rejected_cmds = set()    # MSP commands the drone answered with an error
        self._not_connected_logged = False

        # Developer Mode (Serial Debug)
        self.dev_mode_enabled = False



    def start_write_function(self):
        """Start the background threads: the writer (RC + requests) and the reader (replies)."""
        self._stop_event.clear()
        if self.thread is None or not self.thread.is_alive():
            self.thread = Thread(target=self.write_function, daemon=True)
            self.thread.start()
        if self.reader_thread is None or not self.reader_thread.is_alive():
            self.reader_thread = Thread(target=self._read_function, daemon=True)
            self.reader_thread.start()

    def _join_threads(self, timeout: float = 1.5) -> None:
        """Wait for the background threads of a previous link to finish."""
        for t in (self.thread, self.reader_thread):
            if t is not None and t.is_alive() and t is not current_thread():
                t.join(timeout=timeout)

    def _close_socket(self) -> None:
        if self.client:
            try:
                self.client.shutdown(socket.SHUT_RDWR)  # wakes a thread blocked in recv
            except OSError:
                pass
            try:
                self.client.close()
            except OSError:
                pass

    def _warn_not_connected(self) -> None:
        if not self._not_connected_logged:
            print("Error: Not connected to server")
            self._not_connected_logged = True

    def _link_lost(self, reason) -> None:
        """
        Mark the link as down after a socket error (in a background thread or a send).

        Stops both threads and closes the socket, so `connected` is False and later
        calls report it instead of silently doing nothing. connect() starts a new link.
        """
        with self._lock:
            if not self.connected:
                return  # already handled, or disconnect() is closing the socket
            self.connected = False
        self._stop_event.set()
        print(f"Connection to drone lost ({reason}). Call connect() to reconnect.")
        self._close_socket()
        with self._frames_cv:
            self._frames_cv.notify_all()
        
    def connect(self) -> None:
        """
        Establish a TCP connection to the drone and start the communication thread.
        
        Default IP: 192.168.4.1
        Default Port: 23
        
        Raises:
            socket.timeout: If connection fails to establish within 5 seconds.
        """
        if not self.connected:
            self._join_threads()  # threads of a lost link may still be finishing
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client.settimeout(5.0)  # 5 second timeout for connection
            try:
                self.client.connect((self.TCP_IP, self.TCP_PORT))
                self.client.settimeout(1.0)  # 1 second timeout for reads and sends
                # Nothing from a previous link carries over
                with self._frames_cv:
                    self._frames.clear()
                self._rx_buf.clear()
                self._rx_text.clear()
                self._rejected_cmds.clear()
                self._led_queue.clear()
                self._led_inflight = False
                self._not_connected_logged = False
                self.connected = True
                print(f"Connected to drone at {self.TCP_IP}:{self.TCP_PORT}")
                self.start_write_function()
            except socket.timeout:
                print(f"Connection timeout: Unable to connect to {self.TCP_IP}:{self.TCP_PORT}")
                self.connected = False
                self._close_socket()
            except Exception as e:
                print(f"Error connecting to server: {e}")
                self.connected = False
                self._close_socket()

    def disconnect(self) -> None:
        """
        Disconnect from the drone and stop the background communication thread.
        Cleanly closes the socket and releases resources.
        """
        if self.connected:
            # Let queued LED packets (e.g. a final led_release) go out first
            if self._led_queue or self._led_inflight:
                # ~22 ms per packet; allow 50 ms each for slow timers / Wi-Fi
                self.wait_leds(timeout=0.5 + 0.05 * len(self._led_queue))
            self._led_queue.clear()
            self.connected = False
            self._stop_event.set()  # Signal thread to stop
            
            # Wait for the writer to finish (with timeout)
            if self.thread and self.thread.is_alive():
                self.thread.join(timeout=1.0)

            # Close the socket (this also wakes the reader), then wait for the reader
            self._close_socket()
            if self.reader_thread and self.reader_thread.is_alive():
                self.reader_thread.join(timeout=1.0)
            with self._frames_cv:
                self._frames_cv.notify_all()

            print("Disconnected from drone")

    def cam(self) -> None:
        """
        Configure the connection to use the Camera Module IP/Port.
        
        This should be called BEFORE connect().
        Target IP: 192.168.0.1
        Target Port: 9060
        """
        self.TCP_IP = '192.168.0.1'
        self.TCP_PORT = 9060
        self.camera_mode = True
        
    def arm(self) -> None:
        """
        Arm the drone motors.
        
        Sets the throttle to 1000 and AUX4 to 1500 to arm the flight controller.
        The drone will not spin motors until throttle is raised, but it is ready to fly.
        """
        print("Arming")
        with self._lock:
            self.rcRoll = 1500
            self.rcYaw = 1500
            self.rcPitch = 1500
            self.rcThrottle = 1000
            self.rcAUX4 = 1500

    def box_arm(self) -> None:
        """
        Arm the drone in 'Box Mode'.
        
        Sets throttle to 1800 (high idle?) and AUX4 to 1500.
        Use with caution.
        """
        print("boxarm")
        with self._lock:
            self.rcRoll = 1500
            self.rcYaw = 1500
            self.rcPitch = 1500
            self.rcThrottle = 1800
            self.rcAUX4 = 1500

    def disarm(self) -> None:
        """
        Disarm the drone motors.
        
        Sets AUX4 to 1200 to disarm the flight controller.
        Motors will stop immediately.
        """
        print("Disarm")
        with self._lock:
            self.rcThrottle = 1300
            self.rcAUX4 = 1200

    def devOn(self) -> None:
        """
        Enable Developer Mode (Serial Debugging).
        
        When enabled, the library will automatically intercept and print debug messages
        received from the drone to the terminal. This includes standard MSP debug packets
        and raw serial output.
        """
        print("Developer mode ON")
        with self._lock:
            self.rcAUX2 = 1500
        self.dev_mode_enabled = True
        
    def devOff(self) -> None:
        """
        Disable Developer Mode.
        Stops printing debug messages to the terminal.
        """
        print("Developer mode OFF")
        with self._lock:
            self.rcAUX2 = 1000
        self.dev_mode_enabled = False
        
    def forward(self) -> None:
        """Set Pitch to 1600 (Move Forward)."""
        print("Forward")
        with self._lock:
            self.rcPitch = 1600

    def backward(self) -> None:
        """Set Pitch to 1300 (Move Backward)."""
        print("Backward")
        with self._lock:
            self.rcPitch = 1300

    def left(self) -> None:
        """Set Roll to 1200 (Move Left)."""
        print("Left Roll")
        with self._lock:
            self.rcRoll = 1200

    def right(self) -> None:
        """Set Roll to 1600 (Move Right)."""
        print("Right Roll")
        with self._lock:
            self.rcRoll = 1600

    def left_yaw(self) -> None:
        """Set Yaw to 1300 (Rotate Left)."""
        print("Left Yaw")
        with self._lock:
            self.rcYaw = 1300

    def right_yaw(self) -> None:
        """Set Yaw to 1600 (Rotate Right)."""
        print("Right Yaw")
        with self._lock:
            self.rcYaw = 1600

    def reset(self) -> None:
        """Reset all RC channels to neutral (1500)."""
        print("Reset RC")
        with self._lock:
            self.rcRoll = 1500
            self.rcThrottle = 1500
            self.rcPitch = 1500
            self.rcYaw = 1500
            self.command_type = self.NONE_COMMAND

    def increase_height(self) -> None:
        """Increase Throttle to 1800 (Ascend)."""
        print("Increasing height")
        with self._lock:
            self.rcThrottle = 1800

    def decrease_height(self) -> None:
        """Decrease Throttle to 1300 (Descend)."""
        print("Decreasing height")
        with self._lock:
            self.rcThrottle = 1300

    def take_off(self) -> None:
        """
        Command the drone to Take Off.
        Switches to Auto-Takeoff mode.
        """
        print("Take off")
        self.disarm()
        self.box_arm()
        with self._lock:
            self.command_type = self.TAKE_OFF

    def land(self) -> None:
        """
        Command the drone to Land.
        Switches to Auto-Land mode.
        """
        print("Land")
        with self._lock:
            self.command_type = self.LAND

    def rc_values(self) -> list[int]:
        """Returns the current RC channel values."""
        return [self.rcRoll, self.rcPitch, self.rcThrottle, self.rcYaw,
                self.rcAUX1, self.rcAUX2, self.rcAUX3, self.rcAUX4]

    def motor_speed(self, index: int, value: int) -> None:
        """
        Set the speed of a specific motor.
        
        Args:
            index (int): Motor index (0-3).
            value (int): PWM value (1000-2000).
        """
        print(f"Set motor {index} speed to {value}")
        if 0 <= index < 4:
            # Create a list of 4 motor speeds (defaulting to 1000/Idle)
            # Todo: Ideally reading current state would be better.
            motor_speeds = [1000, 1000, 1000, 1000]
            motor_speeds[index] = value
            
            # create_packet_msp takes a list of 16-bit integers
            self.send_request_msp(self.create_packet_msp(MSP_SET_MOTOR, motor_speeds))
        else:
            print("Invalid motor index. Must be between 0 and 3.")

    # ── WS2812B RGB LEDs ────────────────────────────────────────────────────
    # Needs MagisV2 firmware with MSP 225 / 226. Packets are queued and sent by the
    # background thread at most one per ~22 ms cycle (~45 per second), in place of a
    # telemetry request, so LED traffic never adds to the drone's MSP load. Calls
    # return at once; wait_leds() waits until everything queued has been sent.

    @staticmethod
    def _in_range(name: str, value, low: int, high: int) -> bool:
        if isinstance(value, int) and not isinstance(value, bool) and low <= value <= high:
            return True
        print(f"Invalid {name} {value!r}. Must be an integer {low}-{high}.")
        return False

    def _writer_alive(self) -> bool:
        return self.thread is not None and self.thread.is_alive()

    def _queue_led_packet(self, msp: int, payload: List[int]) -> bool:
        if not self.connected:
            print("Error: Not connected to server")
            return False
        if not self._writer_alive():
            print("Error: background communication thread is not running (connection lost?)")
            return False
        # Back-pressure: a tight loop of set_led() runs at the drone's pace instead of
        # building an ever-growing backlog.
        deadline = time.monotonic() + 1.0
        while len(self._led_queue) >= LED_QUEUE_MAX:
            if time.monotonic() > deadline or not self.connected or not self._writer_alive():
                print("Error: LED queue full, command dropped")
                return False
            time.sleep(0.005)
        self._led_queue.append(self.create_packet_msp_u8(msp, payload))
        return True

    def led_init(self, count: int) -> None:
        """
        Take control of the WS2812B LED strip from the drone.

        Until led_release() or a reboot, the drone's own LED colours and the RGB calls in
        PlutoPilot.cpp are ignored. If PlutoPilot.cpp never called RGB_Init, the drone
        starts the strip on PA8 with `count` LEDs.

        The drone silently ignores led_init (and later set_led calls) when it has to start
        the strip itself and: it is armed, a PPM receiver is used, PA8 is already set up
        by PlutoPilot.cpp (output, PWM or pull-up/down), or the strip's DMA channel is busy.
        Call led_init before arming.

        Args:
            count (int): Number of LEDs on the strip (1-64).
        """
        if self._in_range("LED count", count, 1, RGB_MAX_LEDS):
            self._queue_led_packet(MSP_RGB_LED_CTRL, [1, count])

    def set_led(self, index: int, r: int, g: int, b: int, brightness: int = 100) -> None:
        """
        Set the colour of one LED (needs led_init first).

        Args:
            index (int): LED index (0 to count-1).
            r, g, b (int): Colour, 0-255 each.
            brightness (int): 0-100 %, applied to this colour only (default 100).
        """
        if (self._in_range("LED index", index, 0, RGB_MAX_LEDS - 1)
                and self._in_range("red", r, 0, 255) and self._in_range("green", g, 0, 255)
                and self._in_range("blue", b, 0, 255)
                and self._in_range("brightness", brightness, 0, 100)):
            self._queue_led_packet(MSP_SET_RGB_LED, [index, r, g, b, brightness])

    def set_all_leds(self, r: int, g: int, b: int, brightness: int = 100) -> None:
        """
        Set every LED to one colour with a single packet (needs led_init first).

        Args:
            r, g, b (int): Colour, 0-255 each.
            brightness (int): 0-100 % (default 100).
        """
        if (self._in_range("red", r, 0, 255) and self._in_range("green", g, 0, 255)
                and self._in_range("blue", b, 0, 255)
                and self._in_range("brightness", brightness, 0, 100)):
            self._queue_led_packet(MSP_SET_RGB_LED, [RGB_LED_ALL, r, g, b, brightness])

    def led_release(self) -> None:
        """Turn the strip off and give it back to the drone / PlutoPilot.cpp."""
        self._led_queue.clear()  # colours still queued would be blanked anyway: release goes next
        self._queue_led_packet(MSP_RGB_LED_CTRL, [0, 0])

    def wait_leds(self, timeout: float = 3.0) -> bool:
        """
        Wait until all queued LED packets have been sent.

        Returns:
            bool: True if everything was sent within `timeout` seconds.
        """
        deadline = time.monotonic() + timeout
        while (self._led_queue or self._led_inflight) and self.connected and self._writer_alive():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.01)
        return not self._led_queue and not self._led_inflight

    def trim(self, roll: int, pitch: int) -> None:
        """
        Set ACC Trim values.

        The drone applies them at once and keeps them until it reboots. To keep them
        after a reboot, call send_request_msp_eeprom_write() while disarmed (the drone
        refuses EEPROM writes while armed).

        Args:
            roll (int): Roll trim value (-1000 to 1000).
            pitch (int): Pitch trim value (-1000 to 1000).
        """
        roll = max(min(roll, TRIM_MAX), TRIM_MIN)
        pitch = max(min(pitch, TRIM_MAX), TRIM_MIN)
        self.send_request_msp_set_acc_trim(roll, pitch)
            
    def send_request_msp(self, data) -> bool:
        """Send one MSP packet (hex string). Returns False if it could not be sent."""
        if not self.connected:
            self._warn_not_connected()
            return False
        try:
            # sendall: send() may write part of a packet; the lock keeps packets from the
            # writer thread and from user calls from interleaving on the socket.
            with self._send_lock:
                self.client.sendall(bytes.fromhex(data))
            return True
        except OSError as e:  # includes socket.timeout
            self._link_lost(e)
            return False

    def create_packet_msp(self, msp, payload):
        bf = ""
        bf += MSP_HEADER_IN

        checksum = 0
        if (msp == MSP_SET_COMMAND):
            pl_size = 1
        else:
            pl_size = len(payload) * 2

        bf += '{:02x}'.format(pl_size & 0xFF)
        checksum ^= pl_size

        bf += '{:02x}'.format(msp & 0xFF)
        checksum ^= msp

        for k in payload:
            if (msp == MSP_SET_COMMAND):
                bf += '{:02x}'.format(k & 0xFF)
                checksum ^= k & 0xFF
            else:
                bf += '{:02x}'.format(k & 0xFF)
                checksum ^= k & 0xFF
                bf += '{:02x}'.format((k >> 8) & 0xFF)
                checksum ^= (k >> 8) & 0xFF

        bf += '{:02x}'.format(checksum)

        return bf

    def create_packet_msp_u8(self, msp, payload):
        """MSP v1 packet (hex string) whose payload is bytes (0-255), not 16-bit values."""
        checksum = len(payload) ^ msp
        bf = MSP_HEADER_IN + '{:02x}{:02x}'.format(len(payload) & 0xFF, msp & 0xFF)
        for k in payload:
            bf += '{:02x}'.format(k & 0xFF)
            checksum ^= k & 0xFF
        return bf + '{:02x}'.format(checksum & 0xFF)

    def send_request_msp_set_raw_rc(self, channels):
        self.send_request_msp(self.create_packet_msp(MSP_SET_RAW_RC, channels))

    def send_request_msp_set_command(self, command_type):
        self.send_request_msp(self.create_packet_msp(MSP_SET_COMMAND, [command_type]))

    def send_request_msp_get_debug(self, requests):
        for i in range(len(requests)):
            self.send_request_msp(self.create_packet_msp(requests[i], []))

    def send_request_msp_set_acc_trim(self, trim_roll, trim_pitch):
        # MSP_SET_ACC_TRIM (239), not MSP_ACC_TRIM (240, which only reads the trim).
        # The firmware reads pitch first, then roll.
        self.send_request_msp(self.create_packet_msp(MSP_SET_ACC_TRIM, [trim_pitch, trim_roll]))

    def send_request_msp_acc_trim(self):
        if self.connected:
            self.send_request_msp(self.create_packet_msp(MSP_ACC_TRIM, []))
        else:
            print("Error: Not connected to server")

    def send_request_msp_eeprom_write(self):
        self.send_request_msp(self.create_packet_msp(MSP_EEPROM_WRITE, []))

    def send_request_msp_set_motor(self, motor_speeds):
        self.send_request_msp(self.create_packet_msp(MSP_SET_MOTOR, motor_speeds))

    def write_function(self):
        """Background thread that continuously sends commands to the drone"""
        try:
            self._write_loop()
        except Exception as e:
            # Never die silently: the drone would stop getting RC frames while
            # `connected` still said True.
            self._link_lost(f"writer thread error: {e!r}")

    def _write_loop(self):
        requests = [MSP_RC, MSP_ATTITUDE, MSP_RAW_IMU, MSP_ALTITUDE, MSP_ANALOG]
        led_skip = 0  # telemetry request that gives its slot to the next LED packet
        self.send_request_msp_acc_trim()

        while self.connected and not self._stop_event.is_set():
            # Thread-safe copy of RC values
            with self._lock:
                self.droneRC[:] = self.rc_values()
                cmd_to_send = self.command_type
                if cmd_to_send != self.NONE_COMMAND:
                    self.command_type = self.NONE_COMMAND

            # Send commands (outside lock to avoid blocking user code)
            self.send_request_msp_set_raw_rc(self.droneRC)

            # A queued LED packet replaces one telemetry request (rotating), so this loop
            # still sends 6 packets per cycle. The drone serves one MSP command per 3.5 ms
            # loop (~285/s) and overwrites its 256-byte receive buffer when it falls behind,
            # which would garble the RC frames above.
            cycle_requests = requests
            led_packet = None
            if self._led_queue:
                self._led_inflight = True  # set before popleft, so wait_leds never sees "empty and idle" early
                try:
                    led_packet = self._led_queue.popleft()
                except IndexError:
                    led_packet = None
            if led_packet is not None:
                try:
                    self.send_request_msp(led_packet)
                finally:
                    self._led_inflight = False
                cycle_requests = requests[:led_skip] + requests[led_skip + 1:]
                led_skip = (led_skip + 1) % len(requests)
            else:
                self._led_inflight = False
            self.send_request_msp_get_debug(cycle_requests)
            if cmd_to_send != self.NONE_COMMAND:
                self.send_request_msp_set_command(cmd_to_send)

            time.sleep(0.022)  # ~45Hz update rate

    # ── Telemetry ───────────────────────────────────────────────────────────
    # The writer thread asks for RC, ATTITUDE, RAW_IMU, ALTITUDE and ANALOG every cycle
    # and the reader thread keeps the latest reply of each, so these getters add no MSP
    # traffic and always return current data.

    def get_height(self) -> int:
        """
        Get the current height of the drone.

        Returns:
            int: Estimated altitude in cm (0 if no data).
        """
        height = self._field(MSP_ALTITUDE, 0, 4)  # int32, cm
        if height is None:
            return 0
        print(f"height: {height} cm")
        return height

    def get_vario(self) -> int:
        """
        Get the vertical speed.

        Returns:
            int: Variometer reading in cm/s (0 if no data).
        """
        vario = self._field(MSP_ALTITUDE, 4)
        return 0 if vario is None else vario

    def get_roll(self) -> float:
        """
        Get the current Roll angle.

        Returns:
            float: Roll angle in degrees.
        """
        roll = self._field(MSP_ATTITUDE, 0)  # deci-degrees
        if roll is None:
            return 0.0
        roll = roll / 10
        print(f"roll: {roll} degrees")
        return roll

    def get_pitch(self) -> float:
        """
        Get the current Pitch angle.

        Returns:
            float: Pitch angle in degrees.
        """
        pitch = self._field(MSP_ATTITUDE, 2)  # deci-degrees
        if pitch is None:
            return 0.0
        pitch = pitch / 10
        print(f"pitch: {pitch} degrees")
        return pitch

    def get_yaw(self) -> float:
        """
        Get the current Yaw angle.

        Returns:
            float: Yaw angle in degrees.
        """
        yaw = self._field(MSP_ATTITUDE, 4)  # heading, degrees
        if yaw is None:
            return 0.0
        print(f"yaw: {yaw} degrees")
        return float(yaw)

    def _imu(self, index: int) -> int:
        """RAW_IMU value: 0-2 accelerometer, 3-5 gyroscope, 6-8 magnetometer."""
        value = self._field(MSP_RAW_IMU, index * 2)
        return 0 if value is None else value

    def get_acc_x(self) -> int:
        """Get raw Accelerometer X value (signed 16-bit)."""
        return self._imu(0)

    def get_acc_y(self) -> int:
        """Get raw Accelerometer Y value (signed 16-bit)."""
        return self._imu(1)

    def get_acc_z(self) -> int:
        """Get raw Accelerometer Z value (signed 16-bit)."""
        return self._imu(2)

    def get_gyro_x(self) -> int:
        """Get raw Gyroscope X value (signed 16-bit)."""
        return self._imu(3)

    def get_gyro_y(self) -> int:
        """Get raw Gyroscope Y value (signed 16-bit)."""
        return self._imu(4)

    def get_gyro_z(self) -> int:
        """Get raw Gyroscope Z value (signed 16-bit)."""
        return self._imu(5)

    def get_mag_x(self) -> int:
        """Get raw Magnetometer X value (signed 16-bit)."""
        return self._imu(6)

    def get_mag_y(self) -> int:
        """Get raw Magnetometer Y value (signed 16-bit)."""
        return self._imu(7)

    def get_mag_z(self) -> int:
        """Get raw Magnetometer Z value (signed 16-bit)."""
        return self._imu(8)

    def calibrate_acceleration(self) -> None:
        """
        Calibrate the Accelerometer.
        
        The drone must be placed on a flat, level surface before calling this.
        Motors should be disarmed.
        """
        self.send_request_msp(self.create_packet_msp(MSP_ACC_CALIBRATION, []))

    def calibrate_magnetometer(self) -> None:
        """
        Calibrate the Magnetometer.
        
        Follow the calibration procedure (rotating the drone) after calling this.
        Motors should be disarmed.
        """
        self.send_request_msp(self.create_packet_msp(MSP_MAG_CALIBRATION, []))

    def get_battery(self) -> float:
        """
        Get the current battery voltage.

        Returns:
            float: Battery voltage in Volts (the BMS's compensated voltage on Protocol V1
            boards), 0.0 if no data.
        """
        telemetry = self.get_analog_telemetry()
        if not telemetry:
            return 0.0
        if self.MSP_Protocol_Version == 1:
            return telemetry['vBatComp']
        return telemetry['bytevbat']

    def get_battery_percentage(self) -> float:
        """
        Get the battery percentage.

        On Protocol V1 boards this is the drone's own state of charge from its BMS,
        returned as sent. Older boards do not send one, so it is estimated from the
        voltage (3.7 V = 0 %, 4.2 V = 100 %).

        Returns:
            float: 0.0 to 100.0 (0.0 if no data).
        """
        telemetry = self.get_analog_telemetry()
        if not telemetry:
            return 0.0
        if self.MSP_Protocol_Version == 1:
            return float(telemetry['soc_Fused'])
        voltage = telemetry['bytevbat']
        percentage = ((voltage - BATTERY_IDLE_VOLTAGE)
                      / (BATTERY_MAX_VOLTAGE - BATTERY_IDLE_VOLTAGE)) * 100
        return max(0.0, min(100.0, percentage))

    # ── Receiving ───────────────────────────────────────────────────────────

    def _read_function(self):
        """
        Background thread that drains the socket and parses everything the drone sends.

        Without it the replies to the writer's requests pile up unread and the getters
        would read data that is seconds old.
        """
        silent_seconds = 0
        while self.connected and not self._stop_event.is_set():
            client = self.client
            try:
                data = client.recv(4096)
            except socket.timeout:
                silent_seconds += 1
                if silent_seconds == 3:
                    print("Warning: no data from the drone for 3 seconds (Wi-Fi link down?)")
                continue
            except OSError as e:
                if client is self.client:  # not a socket left over from a previous link
                    self._link_lost(e)
                continue
            if not data:
                if client is self.client:
                    self._link_lost("the drone closed the connection")
                continue
            silent_seconds = 0
            try:
                self._parse_rx(data)
            except Exception as e:  # a parser bug must not stop telemetry for good
                print(f"Telemetry parse error: {e!r}")
                self._rx_buf.clear()

    def _parse_rx(self, data: bytes) -> None:
        """Split received bytes into MSP replies ($M>), debug messages ($D) and serial text."""
        buf = self._rx_buf
        buf += data
        n = len(buf)
        i = 0
        while i < n:
            start = buf.find(b'$', i)
            if start < 0:
                self._serial_text(buf[i:])
                i = n
                break
            if start > i:
                self._serial_text(buf[i:start])
            if n - start < 2:
                i = start  # wait for the byte after '$'
                break
            kind = buf[start + 1]
            if kind == 0x4D:  # 'M': $M> size cmd payload checksum
                if n - start < 5:
                    i = start
                    break
                direction, size, cmd = buf[start + 2], buf[start + 3], buf[start + 4]
                if direction not in (0x3E, 0x21):  # '>' reply, '!' error
                    self._serial_text(buf[start:start + 1])
                    i = start + 1
                    continue
                end = start + 6 + size
                if n < end:
                    i = start
                    break
                payload = bytes(buf[start + 5:end - 1])
                checksum = size ^ cmd
                for b in payload:
                    checksum ^= b
                if checksum != buf[end - 1]:
                    i = start + 1  # corrupt frame: resync on the next '$'
                    continue
                if direction == 0x3E:
                    self._store_frame(cmd, payload)
                elif cmd not in self._rejected_cmds:
                    self._rejected_cmds.add(cmd)
                    print(f"Drone rejected MSP command {cmd} "
                          "(not supported by this firmware, or not allowed right now)")
                i = end
            elif kind == 0x44:  # 'D': $D size message checksum, from debugPrint()
                if n - start < 3:
                    i = start
                    break
                size = buf[start + 2]
                end = start + 4 + size
                if n < end:
                    i = start
                    break
                self._debug_message(bytes(buf[start + 3:end - 1]))
                i = end
            else:
                self._serial_text(buf[start:start + 1])
                i = start + 1
        del buf[:i]

    def _store_frame(self, cmd: int, payload: bytes) -> None:
        with self._frames_cv:
            self._frames[cmd] = (payload, time.monotonic())
            self._frames_cv.notify_all()

    def _get_frame(self, cmd: int, min_size: int, timeout: float = 1.0) -> Optional[bytes]:
        """
        Payload of the latest reply to `cmd`, if it is at most FRAME_MAX_AGE seconds old.

        Otherwise waits up to `timeout` seconds for a new one. Returns None when not
        connected, or when no reply of at least `min_size` bytes arrives in time.
        """
        deadline = time.monotonic() + timeout
        with self._frames_cv:
            while True:
                if not self.connected:  # never hand out data from a link that is gone
                    self._warn_not_connected()
                    return None
                now = time.monotonic()
                frame = self._frames.get(cmd)
                if frame and now - frame[1] <= FRAME_MAX_AGE and len(frame[0]) >= min_size:
                    return frame[0]
                if now >= deadline:
                    return None
                self._frames_cv.wait(deadline - now)

    def _field(self, cmd: int, offset: int, size: int = 2, signed: bool = True) -> Optional[int]:
        """Little-endian integer at payload[offset:offset + size] of the latest `cmd` reply."""
        payload = self._get_frame(cmd, offset + size)
        if payload is None:
            return None
        return int.from_bytes(payload[offset:offset + size], 'little', signed=signed)

    def _serial_text(self, chunk) -> None:
        """Raw text from the drone's serial port: printed line by line in dev mode."""
        if not self.dev_mode_enabled:
            self._rx_text.clear()
            return
        self._rx_text += chunk
        while True:
            newline = self._rx_text.find(b'\n')
            if newline < 0:
                break
            line = self._rx_text[:newline].decode('utf-8', errors='ignore').strip()
            del self._rx_text[:newline + 1]
            if len(line) > 1 and line.isprintable():
                print(f"[SERIAL] {line}")
        if len(self._rx_text) > 1024:  # no newline in sight: not text
            self._rx_text.clear()

    def _debug_message(self, payload: bytes) -> None:
        """A debugPrint() message from the drone: printed in dev mode."""
        if not self.dev_mode_enabled:
            return
        message = payload.decode('utf-8', errors='ignore').strip()
        if message and not message.startswith('~'):
            print(f"[DEBUG {time.strftime('%H:%M:%S')}] {message}")

    def read16(self, arr):
        """Signed little-endian 16-bit value from the first two bytes of arr."""
        return int.from_bytes(bytes(arr[:2]), 'little', signed=True)
    
    def read8(self, value):
        """Read single byte value"""
        return value & 0xFF

    # Enhanced Battery Telemetry Methods
    
    def get_analog_telemetry(self):
        """
        Get the latest MSP_ANALOG (battery) telemetry.

        Protocol V1 boards send the BMS's own values, which are returned as sent:
        current in mA, capacity in mAh, state of charge in %, warning mode as a code.
        Only vBatComp is scaled, from mV to volts.

        Returns:
            dict or None: None if no reply arrived. Keys depend on MSP_Protocol_Version.
        """
        if self.MSP_Protocol_Version == 1:
            # u16 vBatComp (mV), mAmpRaw (mA), mAhDrawn, mAhRemain (mAh); u8 SoC (%), warning mode
            data = self._get_frame(MSP_ANALOG, 10)
            if data is None:
                return None
            self.vBatComp, self.mAmpRaw, self.mAhDrawn, self.mAhRemain = (
                int.from_bytes(data[k:k + 2], 'little') for k in (0, 2, 4, 6))
            self.soc_Fused = data[8]
            self.auto_LandMode = data[9]
            return {
                'vBatComp': self.vBatComp / 1000.0,  # mV -> V
                'mAmpRaw': self.mAmpRaw,
                'mAhDrawn': self.mAhDrawn,
                'mAhRemain': self.mAhRemain,
                'soc_Fused': self.soc_Fused,
                'auto_LandMode': self.auto_LandMode,
                'protocol_version': 1
            }

        # Older boards: u8 vbat (0.1 V), u16 power meter sum, u16 RSSI, s16 amperage
        data = self._get_frame(MSP_ANALOG, 7)
        if data is None:
            return None
        self.bytevbat = data[0]
        self.pMeterSum = int.from_bytes(data[1:3], 'little')
        self.rssi = int.from_bytes(data[3:5], 'little')
        self.amperage = self.read16(data[5:7])
        return {
            'bytevbat': self.bytevbat / 10.0,  # Convert to volts
            'pMeterSum': self.pMeterSum,
            'rssi': self.rssi,
            'amperage': self.amperage,
            'protocol_version': self.MSP_Protocol_Version
        }

    def get_battery_voltage_compensated(self) -> float:
        """Get compensated battery voltage (Volts)."""
        self.get_analog_telemetry()
        if self.MSP_Protocol_Version == 1:
            return self.vBatComp / 1000.0
        else:
            return self.bytevbat / 10.0

    def get_current_mA(self) -> int:
        """Get current draw (mA)."""
        self.get_analog_telemetry()
        if self.MSP_Protocol_Version == 1:
            return self.mAmpRaw
        else:
            return self.amperage

    def get_capacity_drawn_mAh(self) -> int:
        """Get capacity consumed (mAh)."""
        self.get_analog_telemetry()
        return self.mAhDrawn

    def get_capacity_remaining_mAh(self) -> int:
        """Get capacity remaining (mAh)."""
        self.get_analog_telemetry()
        return self.mAhRemain

    def get_state_of_charge(self) -> int:
        """Get State of Charge (0-100%)."""
        self.get_analog_telemetry()
        return self.soc_Fused

    def get_auto_land_status(self) -> int:
        """Get Auto-Land status (1=Active, 0=Inactive)."""
        self.get_analog_telemetry()
        return self.auto_LandMode
    
    def get_rssi(self):
        """Get signal strength RSSI"""
        self.get_analog_telemetry()
        return self.rssi
    
    def get_power_meter_sum(self):
        """Get power meter sum"""
        self.get_analog_telemetry()
        return self.pMeterSum
    
    def get_battery_info(self) -> Optional[Dict[str, Union[str, int, float, str]]]:
        """
        Get comprehensive battery information.
        
        Returns:
            dict: Dictionary containing battery telemetry.
                  Structure depends on MSP_Protocol_Version.
        """
        telemetry = self.get_analog_telemetry()
        if not telemetry:
            return None
            
        if self.MSP_Protocol_Version == 1:
             return {
                'voltage': f"{telemetry['vBatComp']:.2f}V",
                'current': f"{telemetry['mAmpRaw']}mA",
                'capacity_drawn': f"{telemetry['mAhDrawn']}mAh",
                'capacity_remaining': f"{telemetry['mAhRemain']}mAh",
                'state_of_charge': f"{telemetry['soc_Fused']}%",
                'auto_land_mode': telemetry['auto_LandMode'],
                'protocol': "V1 (Enhanced)"
            }
        else:
             return {
                'voltage': f"{telemetry['bytevbat']:.2f}V",
                'current': f"{telemetry.get('amperage', 0)}mA",
                'rssi': telemetry['rssi'],
                'power_sum': telemetry['pMeterSum'],
                'protocol': "Legacy"
            }
    
    def get_battery_level_status(self) -> str:
        """
        Get battery level status based on state of charge.
        
        Returns:
            str: 'FULL', 'TWO_BAR', 'ONE_BAR', or 'EMPTY'.
        """
        soc = self.get_state_of_charge()
        
        if soc >= 70:
            return 'FULL'  # 4 bars - battery_full
        elif soc >= 40:
            return 'TWO_BAR'  # 2 bars - battery_two_bar
        elif soc >= 20:
            return 'ONE_BAR'  # 1 bar - battery_one_bar
        else:
            return 'EMPTY'  # Empty - no_battery
    
    def get_auto_land_status_text(self) -> str:
        """
        Get auto-land mode as human-readable text.
        
        Returns:
            str: 'NORMAL', 'LOW_BATTERY_WARNING', or 'AUTO_LANDING'.
        """
        auto_land = self.get_auto_land_status()
        
        if auto_land == 0:
            return 'NORMAL'
        elif auto_land == 1:
            return 'LOW_BATTERY_WARNING'
        elif auto_land == 2:
            return 'AUTO_LANDING'
        else:
            return f'UNKNOWN_{auto_land}'
    
    def is_battery_critical(self) -> bool:
        """
        Check if battery is in critical state (< 20% SOC).
        
        Returns:
            bool: True if critical, False otherwise.
        """
        soc = self.get_state_of_charge()
        return soc < 20
    
    def is_battery_low(self) -> bool:
        """
        Check if battery is low (< 40% SOC).
        
        Returns:
            bool: True if low, False otherwise.
        """
        soc = self.get_state_of_charge()
        return soc < 40
    
    def should_auto_land(self) -> bool:
        """
        Check if auto-land is active (mode == 2).
        
        Returns:
            bool: True if auto-landing is triggered.
        """
        return self.auto_LandMode == 2
    
    def get_battery_status_summary(self) -> Dict[str, Union[str, bool, List[str]]]:
        """
        Get comprehensive battery status summary matching mobile app display.
        
        Returns:
            dict: Detailed status including icon, warnings, and flags.
        """
        if self.MSP_Protocol_Version != 1:
            return {
                'error': 'Enhanced telemetry only available in Protocol V1',
                'voltage': self.get_battery_voltage_compensated(),
                'protocol': self.MSP_Protocol_Version
            }
        
        voltage = self.get_battery_voltage_compensated()
        soc = self.get_state_of_charge()
        capacity_remaining = self.get_capacity_remaining_mAh()
        battery_level = self.get_battery_level_status()
        auto_land_status = self.get_auto_land_status_text()
        
        # Determine status icon/emoji
        if battery_level == 'FULL':
            icon = '🔋'  # Full battery
        elif battery_level == 'TWO_BAR':
            icon = '🔋'  # Medium battery
        elif battery_level == 'ONE_BAR':
            icon = '🪫'  # Low battery
        else:
            icon = '🪫'  # Empty battery
        
        # Determine warning message
        warnings = []
        if auto_land_status == 'AUTO_LANDING':
            warnings.append('⚠️ AUTO LANDING IN PROGRESS!')
        elif auto_land_status == 'LOW_BATTERY_WARNING':
            warnings.append('⚠️ LOW BATTERY - Please Land')
        
        if soc < 20:
            warnings.append('🔴 CRITICAL - Land Immediately!')
        elif soc < 40:
            warnings.append('🟡 LOW - Consider Landing')
        
        return {
            'icon': icon,
            'voltage': f"{voltage:.2f}V",
            'soc': f"{soc}%",
            'capacity_remaining': f"{capacity_remaining}mAh",
            'battery_level': battery_level,
            'auto_land_status': auto_land_status,
            'warnings': warnings,
            'is_critical': self.is_battery_critical(),
            'is_low': self.is_battery_low(),
            'should_auto_land': self.should_auto_land()
        }
