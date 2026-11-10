import socket
from collections import deque
from threading import Thread, Lock, Event, Condition, current_thread
import time
from typing import Optional, List, Dict, Any

TRIM_MAX = 1000
TRIM_MIN = -1000
MSP_HEADER_IN = "244d3c"

# RC sticks
RC_MIN = 1000
RC_CENTER = 1500
RC_MAX = 2000
MOVE_STRENGTH_DEFAULT = 200  # MagisV2: offset from centre for forward()/left()/left_yaw()/..., same in every direction
MOVE_STRENGTH_MAX = 500      # 1500 +/- 500 is the full stick range
# Older Magis firmware keeps the original (uneven) default offsets
LEGACY_MOVE_OFFSET = {'forward': 100, 'backward': 200, 'left': 300, 'right': 100,
                      'left_yaw': 200, 'right_yaw': 100}
ARM_AUX_MIN = 1300           # AUX4 at or above this arms (MagisV2 ARM range 1300-2100)
LINK_TIMEOUT = 3.0           # seconds without any data from the drone: the link is lost

# Battery Constants (1S LiPo): only for the voltage estimate on older Magis boards.
# MagisV2 sends its own state of charge and mAh, computed with the capacity stored in
# the drone, so the library does no battery maths for it.
BATTERY_MAX_VOLTAGE = 4.2  # Fully charged
BATTERY_IDLE_VOLTAGE = 3.7  # Nominal/idle
# MagisV2 battery warning code, the last byte of MSP_ANALOG
BATTERY_WARNING_NAMES = {0: 'OK', 1: 'WARNING', 2: 'CRITICAL'}

# MSP Protocol Commands

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
MSP_SET_COMMAND = 217
FRAME_MAX_AGE = 0.5  # seconds: getters wait for a newer reply than this

# WS2812B RGB LED strip (needs MagisV2 firmware with MSP 225 / 226)
MSP_SET_RGB_LED = 225   # [index, r, g, b, brightness %], index 0xFF = all LEDs
MSP_RGB_LED_CTRL = 226  # [op, count]: op 1 = take the strip, op 0 = release
RGB_LED_ALL = 0xFF
RGB_MAX_LEDS = 64
LED_QUEUE_MAX = 64      # queued LED packets before set_led() waits for the drone

# Altitude-hold target (needs MagisV2 firmware with MSP 227)
MSP_SET_ALT_HOLD = 227  # [int32 cm]: same as setDesiredPosition(Z, cm) in PlutoPilot.cpp
HEIGHT_MIN_CM = 20      # set_height: lower targets would sit armed on the floor (land() goes down)
HEIGHT_MAX_CM = 10000   # 100 m: catches values passed in mm by mistake

MSP_MOTOR_COUNT = 8     # MSP_SET_MOTOR always carries 8 motors (the firmware reads 8)

# Flight status and height ceiling (MagisV2)
MSP_MAX_ALT = 123        # reply: int16 ceiling in cm, -1 = none
MSP_SET_MAX_ALT = 218    # [int16 cm], -1 = no ceiling; applies in altitude hold, until reboot
MSP_FLIGHT_STATUS = 255  # reply: uint16 with one bit set, the drone's highest-priority state
FLIGHT_STATUS_NAMES = ['ACCEL_GYRO_CALIBRATION', 'MAG_CALIBRATION', 'ARMED', 'OK_TO_ARM',
                       'NOT_OK_TO_ARM', 'SIGNAL_LOSS', 'CRASH', 'LOW_BATTERY',
                       'LOW_BATTERY_IN_FLIGHT']
REQUEST_REUSE_AGE = 0.1  # s: on-demand replies younger than this are reused (<= 10 requests/s)

# Battery capacity stored in the drone (MagisV2)
MSP_VOLTAGE_METER_CONFIG = 56      # reply: u8 max, u8 min, u8 warning (0.1 V per cell), u16 capacity mAh
MSP_SET_VOLTAGE_METER_CONFIG = 57  # [u8 max, u8 warning, u8 min, u16 capacity mAh]: warning before min
BATTERY_CAPACITY_MIN = 100   # mAh
BATTERY_CAPACITY_MAX = 5000  # mAh

# Why the drone may refuse these ($M!). Firmware without the plutocontrol extensions
# (225-227) refuses them as unknown and nothing else happens; other functions still work.
REJECT_HINTS = {
    MSP_SET_RGB_LED: "LED colour: needs firmware with MSP 225/226, and led_init() first",
    MSP_RGB_LED_CTRL: "LED strip: needs firmware with MSP 225/226; disarm first, PA8 must be free",
    MSP_SET_ALT_HOLD: ("set_height: needs firmware with MSP 227, and works only while flying "
                       "in altitude hold (after take-off, not while landing or in a flip)"),
    MSP_EEPROM_WRITE: "EEPROM write: refused while armed",
}

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
        self.camera_mode = False
        self.TCP_IP = ip
        self.TCP_PORT = port
        
        # MSP Protocol Version (Pluto drones use Protocol V1)
        # 1 = MagisV2 battery layout (BMS values), 0 = older Magis 7-byte layout.
        # Set automatically from every MSP_ANALOG reply (see get_analog_telemetry).
        self.MSP_Protocol_Version = 1
        
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
        self._pending_height = None  # set_height() target (cm) not sent yet; a newer call replaces it
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
        self._fw_v2 = None             # True: MagisV2, False: older Magis, None: no MSP_ANALOG reply yet
        self._motor_test_used = False  # motor_speed() left non-idle values on a MagisV2 drone

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
        self._clear_battery_cache()
        with self._frames_cv:
            self._frames_cv.notify_all()
        
    def connect(self) -> bool:
        """
        Establish a TCP connection to the drone and start the communication thread.

        Default IP: 192.168.4.1
        Default Port: 23

        Errors are printed, not raised (for example no answer within 5 seconds).

        Returns:
            bool: True if connected (or already connected), False if it failed.
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
                self._pending_height = None
                self._not_connected_logged = False
                self._fw_v2 = None  # learnt again from the first MSP_ANALOG reply
                self._clear_battery_cache()
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
        return self.connected

    def _magis_v2(self) -> bool:
        """
        True for MagisV2 firmware (10-byte MSP_ANALOG reply), False for older Magis (7 bytes).

        Right after connect() it waits up to 1 s for the first reply. Without one (not
        connected, or no telemetry) it assumes MagisV2, the library's default.
        """
        if self._fw_v2 is None and self.connected:
            self._get_frame(MSP_ANALOG, 7)  # the reader thread sets _fw_v2
        return self._fw_v2 is not False

    def _clear_motor_test(self) -> None:
        """MagisV2 keeps motor_speed() values and spins them whenever disarmed: set all idle."""
        if self._motor_test_used and self._magis_v2():
            self.send_request_msp_set_motor([RC_MIN] * 4)
            self._motor_test_used = False

    def disconnect(self) -> None:
        """
        Disconnect from the drone and stop the background communication thread.
        Cleanly closes the socket and releases resources.

        On MagisV2, motors left spinning by motor_speed() are set back to idle first.
        """
        if self.connected:
            self._clear_motor_test()
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
            self._clear_battery_cache()

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
        self._clear_motor_test()
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
        self._clear_motor_test()
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
        
    # Movement: each call holds its stick at 1500 +/- strength until reset() or another
    # call. On MagisV2 every direction defaults to 200, so opposite moves are equally
    # strong. Older Magis firmware keeps its original defaults (LEGACY_MOVE_OFFSET).

    @staticmethod
    def _in_range(name: str, value, low: int, high: int) -> bool:
        if isinstance(value, int) and not isinstance(value, bool) and low <= value <= high:
            return True
        print(f"Invalid {name} {value!r}. Must be an integer {low}-{high}.")
        return False

    def _move_offset(self, move: str, strength: Optional[int]) -> Optional[int]:
        """Stick offset for a move, or None (with a message) if `strength` is invalid."""
        if strength is None:
            return MOVE_STRENGTH_DEFAULT if self._magis_v2() else LEGACY_MOVE_OFFSET[move]
        if self._in_range("strength", strength, 0, MOVE_STRENGTH_MAX):
            return strength
        return None

    def forward(self, strength: Optional[int] = None) -> None:
        """Move forward: pitch = 1500 + strength (int 0-500; default 200 on MagisV2, 100 on older Magis)."""
        offset = self._move_offset('forward', strength)
        if offset is not None:
            print("Forward")
            with self._lock:
                self.rcPitch = RC_CENTER + offset

    def backward(self, strength: Optional[int] = None) -> None:
        """Move backward: pitch = 1500 - strength (int 0-500; default 200)."""
        offset = self._move_offset('backward', strength)
        if offset is not None:
            print("Backward")
            with self._lock:
                self.rcPitch = RC_CENTER - offset

    def left(self, strength: Optional[int] = None) -> None:
        """Move left: roll = 1500 - strength (int 0-500; default 200 on MagisV2, 300 on older Magis)."""
        offset = self._move_offset('left', strength)
        if offset is not None:
            print("Left Roll")
            with self._lock:
                self.rcRoll = RC_CENTER - offset

    def right(self, strength: Optional[int] = None) -> None:
        """Move right: roll = 1500 + strength (int 0-500; default 200 on MagisV2, 100 on older Magis)."""
        offset = self._move_offset('right', strength)
        if offset is not None:
            print("Right Roll")
            with self._lock:
                self.rcRoll = RC_CENTER + offset

    def left_yaw(self, strength: Optional[int] = None) -> None:
        """Rotate left: yaw = 1500 - strength (int 0-500; default 200)."""
        offset = self._move_offset('left_yaw', strength)
        if offset is not None:
            print("Left Yaw")
            with self._lock:
                self.rcYaw = RC_CENTER - offset

    def right_yaw(self, strength: Optional[int] = None) -> None:
        """Rotate right: yaw = 1500 + strength (int 0-500; default 200 on MagisV2, 100 on older Magis)."""
        offset = self._move_offset('right_yaw', strength)
        if offset is not None:
            print("Right Yaw")
            with self._lock:
                self.rcYaw = RC_CENTER + offset

    def reset(self) -> None:
        """
        Reset roll, pitch, yaw and throttle to neutral (1500).

        A take_off() or land() that has not gone out yet is still sent.
        """
        print("Reset RC")
        with self._lock:
            self.rcRoll = 1500
            self.rcThrottle = 1500
            self.rcPitch = 1500
            self.rcYaw = 1500

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

        MagisV2: arms with the sticks centred (throttle 1500), which the firmware needs
        to take off; the drone then climbs to its take-off height (120 cm) and holds it.
        Older Magis: throttle 1800, as before.
        """
        print("Take off")
        if not self._magis_v2():
            self.disarm()
            self.box_arm()
            with self._lock:
                self.command_type = self.TAKE_OFF
            return
        self._clear_motor_test()
        # One update, so the writer never sends a frame in between (e.g. a disarm)
        with self._lock:
            self.rcRoll = RC_CENTER
            self.rcPitch = RC_CENTER
            self.rcYaw = RC_CENTER
            self.rcThrottle = RC_CENTER
            self.rcAUX4 = 1500
            self.command_type = self.TAKE_OFF

    def land(self) -> None:
        """
        Command the drone to Land.
        Switches to Auto-Land mode.
        """
        print("Land")
        with self._lock:
            self.command_type = self.LAND

    def rc_values(self) -> List[int]:
        """Returns the current RC channel values."""
        return [self.rcRoll, self.rcPitch, self.rcThrottle, self.rcYaw,
                self.rcAUX1, self.rcAUX2, self.rcAUX3, self.rcAUX4]

    def motor_speed(self, index: int, value: int) -> None:
        """
        Set the speed of a specific motor.
        
        Args:
            index (int): Motor index (0-3).
            value (int): PWM value (1000-2000).

        Anything else (out of range, float, bool, str) is refused with a message and
        nothing is sent.

        This is a motor test: it spins the motor only while the drone is disarmed, and
        the motor keeps spinning until motor_speed(index, 1000), arm(), take_off() or
        disconnect(). On MagisV2 it is refused while armed, because the drone would keep
        the value and spin the motor as soon as it disarms.
        """
        if not (self._in_range("motor index", index, 0, 3)
                and self._in_range("motor speed", value, RC_MIN, RC_MAX)):
            return
        if self._refuse_if_armed("motor_speed"):
            return
        magis_v2 = self._magis_v2()
        print(f"Set motor {index} speed to {value}")
        # The other motors are sent as 1000 (idle). Todo: keep their current values instead.
        motor_speeds = [RC_MIN] * 4
        motor_speeds[index] = value
        self.send_request_msp_set_motor(motor_speeds)  # padded to the 8 motors the firmware reads
        if magis_v2:
            self._motor_test_used = value != RC_MIN

    # ── WS2812B RGB LEDs ────────────────────────────────────────────────────
    # Needs MagisV2 firmware with MSP 225 / 226. Packets are queued and sent by the
    # background thread at most one per ~22 ms cycle (~45 per second), in place of a
    # telemetry request, so LED traffic never adds to the drone's MSP load. Calls
    # return at once; wait_leds() waits until everything queued has been sent.

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

    # ── Altitude hold target ────────────────────────────────────────────────

    def set_height(self, height_cm: int) -> None:
        """
        Set the height the drone holds, in cm (needs MagisV2 firmware with MSP 227).

        Works like setDesiredPosition(Z, cm) in PlutoPilot.cpp: the drone climbs or
        descends to the new height smoothly and holds it. Call it after take-off: take-off
        sets its own target (120 cm) and replaces an earlier one. Keep the throttle
        centred (take_off() and reset() leave it at 1500): moving the throttle stick
        (increase_height(), decrease_height()) takes over from the target. A
        max-altitude ceiling still applies.

        Sent by the background thread in place of one telemetry request, like the LED
        commands. Calling it again before it has gone out replaces the pending value, so
        calling it in a loop never builds a backlog.

        Args:
            height_cm (int): Target height in cm (20-10000).

        The drone takes it only while flying in altitude hold: not before take-off, while
        landing or during a flip (it answers with an error, printed once). A target above
        the set_max_height() ceiling is lowered under it.
        """
        if not self._in_range("height", height_cm, HEIGHT_MIN_CM, HEIGHT_MAX_CM):
            return
        if not self.connected:
            self._warn_not_connected()
            return
        with self._lock:
            self._pending_height = height_cm

    def trim(self, roll: int, pitch: int) -> None:
        """
        Set ACC Trim values.

        The drone applies them at once and keeps them until it reboots. To keep them
        after a reboot, call send_request_msp_eeprom_write() while disarmed (the drone
        refuses EEPROM writes while armed).

        Args:
            roll (int): Roll trim value (-1000 to 1000).
            pitch (int): Pitch trim value (-1000 to 1000).

        Anything else (out of range, float, bool, str) is refused with a message and
        nothing is sent.
        """
        if not (self._in_range("roll trim", roll, TRIM_MIN, TRIM_MAX)
                and self._in_range("pitch trim", pitch, TRIM_MIN, TRIM_MAX)):
            return
        self.send_request_msp_set_acc_trim(roll, pitch)
            
    def send_request_msp(self, data) -> bool:
        """Send one MSP packet (hex string). Returns False if it could not be sent."""
        client = self.client
        if not self.connected or client is None:
            self._warn_not_connected()
            return False
        try:
            # sendall: send() may write part of a packet; the lock keeps packets from the
            # writer thread and from user calls from interleaving on the socket.
            with self._send_lock:
                client.sendall(bytes.fromhex(data))
            return True
        except OSError as e:  # includes socket.timeout
            self._link_lost(e)
            return False

    def create_packet_msp(self, msp, payload):
        """MSP v1 packet (hex string) whose payload is 16-bit little-endian values."""
        bf = ""
        bf += MSP_HEADER_IN

        checksum = 0
        pl_size = len(payload) * 2

        bf += '{:02x}'.format(pl_size & 0xFF)
        checksum ^= pl_size & 0xFF

        bf += '{:02x}'.format(msp & 0xFF)
        checksum ^= msp & 0xFF

        for k in payload:
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
        # 16-bit, because every Magis / MagisV2 firmware reads it with read16 ( ). A 1-byte
        # payload only worked because current_command happens to be uint8 there.
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
        # The firmware always reads 8 motors; pad with idle so motors 5-8 never get
        # leftover bytes from an earlier packet.
        speeds = list(motor_speeds)[:MSP_MOTOR_COUNT]
        speeds += [1000] * (MSP_MOTOR_COUNT - len(speeds))
        self.send_request_msp(self.create_packet_msp(MSP_SET_MOTOR, speeds))

    def write_function(self):
        """Background thread that continuously sends commands to the drone"""
        try:
            self._write_loop()
        except Exception as e:
            # Never die silently: the drone would stop getting RC frames while
            # `connected` still said True.
            self._link_lost(f"writer thread error: {e!r}")

    def _write_loop(self):
        # MSP_RC's reply is never read. MagisV2 drones are not asked for it, which leaves
        # them more room (they serve one MSP command per 3.5 ms); older Magis still are.
        requests_v2 = [MSP_ATTITUDE, MSP_RAW_IMU, MSP_ALTITUDE, MSP_ANALOG]
        requests_legacy = [MSP_RC] + requests_v2
        slot_skip = 0  # telemetry request that gives its slot to the next height / LED packet
        self.send_request_msp_acc_trim()

        while self.connected and not self._stop_event.is_set():
            requests = requests_v2 if self._fw_v2 else requests_legacy
            slot_skip %= len(requests)
            # Thread-safe copy of RC values
            with self._lock:
                self.droneRC[:] = self.rc_values()
                cmd_to_send = self.command_type
                if cmd_to_send != self.NONE_COMMAND:
                    self.command_type = self.NONE_COMMAND
                height_to_send = self._pending_height
                self._pending_height = None

            # Send commands (outside lock to avoid blocking user code)
            self.send_request_msp_set_raw_rc(self.droneRC)

            # A height target or a queued LED packet replaces one telemetry request
            # (rotating), so this loop never sends more packets per cycle. The drone serves one
            # MSP command per 3.5 ms loop (~285/s) and overwrites its 256-byte receive buffer
            # when it falls behind, which would garble the RC frames above.
            cycle_requests = requests
            slot_packet = None
            if height_to_send is not None:
                # Height goes first: it matters for flight, LED colours can wait a cycle
                slot_packet = self.create_packet_msp_u8(
                    MSP_SET_ALT_HOLD, list(height_to_send.to_bytes(4, 'little', signed=True)))
            elif self._led_queue:
                self._led_inflight = True  # set before popleft, so wait_leds never sees "empty and idle" early
                try:
                    slot_packet = self._led_queue.popleft()
                except IndexError:
                    slot_packet = None
            if slot_packet is not None:
                try:
                    self.send_request_msp(slot_packet)
                finally:
                    self._led_inflight = False
                cycle_requests = requests[:slot_skip] + requests[slot_skip + 1:]
                slot_skip = (slot_skip + 1) % len(requests)
            else:
                self._led_inflight = False
            self.send_request_msp_get_debug(cycle_requests)
            if cmd_to_send != self.NONE_COMMAND:
                self.send_request_msp_set_command(cmd_to_send)

            time.sleep(0.022)  # ~45Hz update rate

    # ── Telemetry ───────────────────────────────────────────────────────────
    # The writer thread asks for ATTITUDE, RAW_IMU, ALTITUDE and ANALOG every cycle and
    # the reader thread keeps the latest reply of each, so these getters add no MSP
    # traffic and always return current data. Without current data (not connected, or
    # nothing from the drone for 1 s) they return None, never a made-up 0.

    def get_height(self) -> Optional[int]:
        """
        Get the current height of the drone.

        Returns:
            int or None: Estimated altitude in cm, None if no data.
        """
        height = self._field(MSP_ALTITUDE, 0, 4)  # int32, cm
        if height is None:
            return None
        print(f"height: {height} cm")
        return height

    def get_vario(self) -> Optional[int]:
        """
        Get the vertical speed.

        Returns:
            int or None: Variometer reading in cm/s, None if no data.
        """
        return self._field(MSP_ALTITUDE, 4)

    def get_roll(self) -> Optional[float]:
        """
        Get the current Roll angle.

        Returns:
            float or None: Roll angle in degrees, None if no data.
        """
        roll = self._field(MSP_ATTITUDE, 0)  # deci-degrees
        if roll is None:
            return None
        roll = roll / 10
        print(f"roll: {roll} degrees")
        return roll

    def get_pitch(self) -> Optional[float]:
        """
        Get the current Pitch angle.

        Returns:
            float or None: Pitch angle in degrees, None if no data.
        """
        pitch = self._field(MSP_ATTITUDE, 2)  # deci-degrees
        if pitch is None:
            return None
        pitch = pitch / 10
        print(f"pitch: {pitch} degrees")
        return pitch

    def get_yaw(self) -> Optional[float]:
        """
        Get the current Yaw angle.

        Returns:
            float or None: Yaw angle in degrees, None if no data.
        """
        yaw = self._field(MSP_ATTITUDE, 4)  # heading, degrees
        if yaw is None:
            return None
        print(f"yaw: {yaw} degrees")
        return float(yaw)

    def _imu(self, index: int) -> Optional[int]:
        """RAW_IMU value: 0-2 accelerometer, 3-5 gyroscope, 6-8 magnetometer. None if no data."""
        return self._field(MSP_RAW_IMU, index * 2)

    def get_acc_x(self) -> Optional[int]:
        """Get raw Accelerometer X value (signed 16-bit), None if no data."""
        return self._imu(0)

    def get_acc_y(self) -> Optional[int]:
        """Get raw Accelerometer Y value (signed 16-bit), None if no data."""
        return self._imu(1)

    def get_acc_z(self) -> Optional[int]:
        """Get raw Accelerometer Z value (signed 16-bit), None if no data."""
        return self._imu(2)

    def get_gyro_x(self) -> Optional[int]:
        """Get raw Gyroscope X value (signed 16-bit), None if no data."""
        return self._imu(3)

    def get_gyro_y(self) -> Optional[int]:
        """Get raw Gyroscope Y value (signed 16-bit), None if no data."""
        return self._imu(4)

    def get_gyro_z(self) -> Optional[int]:
        """Get raw Gyroscope Z value (signed 16-bit), None if no data."""
        return self._imu(5)

    def get_mag_x(self) -> Optional[int]:
        """Get raw Magnetometer X value (signed 16-bit), None if no data."""
        return self._imu(6)

    def get_mag_y(self) -> Optional[int]:
        """Get raw Magnetometer Y value (signed 16-bit), None if no data."""
        return self._imu(7)

    def get_mag_z(self) -> Optional[int]:
        """Get raw Magnetometer Z value (signed 16-bit), None if no data."""
        return self._imu(8)

    def _refuse_if_armed(self, what: str) -> bool:
        """
        MagisV2: True (with a message) if the drone is armed, i.e. AUX4 was last set in
        the ARM range. The library is the drone's only RC source, so below it the drone
        cannot be armed.
        """
        if self._magis_v2() and self.rcAUX4 >= ARM_AUX_MIN:
            print(f"{what} refused: the drone is armed. Call disarm() first.")
            return True
        return False

    def calibrate_acceleration(self) -> None:
        """
        Calibrate the Accelerometer.

        The drone must be placed on a flat, level surface and kept still for a few
        seconds while it calibrates.

        The drone ignores this while armed, without an error. On MagisV2 it is therefore
        refused while armed: call disarm() first.
        """
        if not self._refuse_if_armed("calibrate_acceleration"):
            self.send_request_msp(self.create_packet_msp(MSP_ACC_CALIBRATION, []))

    def calibrate_magnetometer(self) -> None:
        """
        Calibrate the Magnetometer.

        Follow the calibration procedure (rotating the drone) after calling this.

        The drone ignores this while armed, without an error. On MagisV2 it is therefore
        refused while armed: call disarm() first.
        """
        if not self._refuse_if_armed("calibrate_magnetometer"):
            self.send_request_msp(self.create_packet_msp(MSP_MAG_CALIBRATION, []))

    # ── Flight status and height ceiling (MagisV2) ──────────────────────────

    def _requires_magis_v2(self, what: str) -> bool:
        if self._magis_v2():
            return True
        print(f"{what}: needs MagisV2 firmware")
        return False

    def _request(self, cmd: int, min_size: int) -> Optional[bytes]:
        """
        Ask the drone for `cmd` (not in the writer's telemetry rotation) and wait up to 1 s.

        A reply younger than REQUEST_REUSE_AGE is reused, so calling in a loop adds at
        most ~10 requests per second to the drone's MSP load.
        """
        with self._frames_cv:
            frame = self._frames.get(cmd)
            if frame and time.monotonic() - frame[1] <= REQUEST_REUSE_AGE and len(frame[0]) >= min_size:
                return frame[0]
            self._frames.pop(cmd, None)  # only the reply to this request counts
        if not self.send_request_msp(self.create_packet_msp(cmd, [])):
            return None
        return self._get_frame(cmd, min_size)

    def get_flight_status(self) -> Optional[str]:
        """
        The drone's flight status, as the Pluto app shows it (MagisV2).

        The drone reports its single most important state, in this order: calibrating,
        crash, low battery, signal loss, not OK to arm, OK to arm, armed. Use it to find
        out why arm() or take_off() did nothing.

        Returns:
            str or None: 'ACCEL_GYRO_CALIBRATION', 'MAG_CALIBRATION', 'CRASH',
            'LOW_BATTERY', 'LOW_BATTERY_IN_FLIGHT', 'SIGNAL_LOSS', 'NOT_OK_TO_ARM',
            'OK_TO_ARM' or 'ARMED'; None if no reply.
        """
        if not self._requires_magis_v2("get_flight_status"):
            return None
        data = self._request(MSP_FLIGHT_STATUS, 2)
        if data is None:
            return None
        flags = int.from_bytes(data[:2], 'little')
        if flags == 0:
            return 'UNKNOWN'
        bit = (flags & -flags).bit_length() - 1
        return FLIGHT_STATUS_NAMES[bit] if bit < len(FLIGHT_STATUS_NAMES) else f'UNKNOWN_{flags}'

    def is_armed(self) -> Optional[bool]:
        """
        Whether the drone is armed, from its flight status (MagisV2).

        Returns:
            bool or None: True if it reports ARMED, False if it reports OK_TO_ARM,
            NOT_OK_TO_ARM or a calibration. None if there is no reply, or a more important
            state (crash, low battery, signal loss) hides whether it is armed.
        """
        status = self.get_flight_status()
        if status == 'ARMED':
            return True
        if status in ('OK_TO_ARM', 'NOT_OK_TO_ARM', 'ACCEL_GYRO_CALIBRATION', 'MAG_CALIBRATION'):
            return False
        return None

    def set_max_height(self, height_cm: Optional[int]) -> None:
        """
        Set a height ceiling in cm (MagisV2), or remove it with None.

        In altitude hold the drone does not climb above it. The drone keeps it until it
        reboots.

        Args:
            height_cm (int or None): Ceiling in cm (0-10000), None for no ceiling.
        """
        if height_cm is not None and not self._in_range("max height", height_cm, 0, HEIGHT_MAX_CM):
            return
        if self._requires_magis_v2("set_max_height"):
            value = -1 if height_cm is None else height_cm
            self.send_request_msp(self.create_packet_msp(MSP_SET_MAX_ALT, [value]))

    def get_max_height(self) -> Optional[int]:
        """
        The drone's height ceiling in cm (MagisV2).

        Returns:
            int or None: Ceiling in cm, -1 if there is none, None if no reply.
        """
        if not self._requires_magis_v2("get_max_height"):
            return None
        data = self._request(MSP_MAX_ALT, 2)
        return None if data is None else int.from_bytes(data[:2], 'little', signed=True)

    def get_battery(self) -> Optional[float]:
        """
        Get the current battery voltage.

        Returns:
            float or None: Battery voltage in Volts (the BMS's compensated voltage on
            Protocol V1 boards), None if no data.
        """
        telemetry = self.get_analog_telemetry()
        if not telemetry:
            return None
        if self.MSP_Protocol_Version == 1:
            return telemetry['vBatComp']
        return telemetry['bytevbat']

    def get_battery_percentage(self) -> Optional[float]:
        """
        Get the battery percentage.

        On Protocol V1 boards this is the drone's own state of charge from its BMS,
        returned as sent. Older boards do not send one, so it is estimated from the
        voltage (3.7 V = 0 %, 4.2 V = 100 %).

        Returns:
            float or None: 0.0 to 100.0, None if no data.
        """
        telemetry = self.get_analog_telemetry()
        if not telemetry:
            return None
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
        last_data = time.monotonic()
        got_data = False  # the drone has answered on this link
        warned = False
        while self.connected and not self._stop_event.is_set():
            client = self.client
            if client is None:
                break
            try:
                data = client.recv(4096)
            except socket.timeout:
                if time.monotonic() - last_data < LINK_TIMEOUT or client is not self.client:
                    continue
                if got_data:
                    # A Wi-Fi drop often closes nothing: TCP would keep the link "up" for
                    # minutes while the drone hears nothing. The drone answered every cycle
                    # until now, so silence this long means the link is gone.
                    self._link_lost(f"no data from the drone for {LINK_TIMEOUT:.0f} s, Wi-Fi link down?")
                elif not warned:
                    # Never answered: keep sending (a bridge might not relay replies)
                    print(f"Warning: no data from the drone for {LINK_TIMEOUT:.0f} seconds (Wi-Fi link down?)")
                    warned = True
                continue
            except OSError as e:
                if client is self.client:  # not a socket left over from a previous link
                    self._link_lost(e)
                continue
            if not data:
                if client is self.client:
                    self._link_lost("the drone closed the connection")
                continue
            last_data = time.monotonic()
            got_data = True
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
                    self._report_rejected(cmd)
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

    def _report_rejected(self, cmd: int) -> None:
        """Say once per connection that the drone refused `cmd` ($M!), and why it may have."""
        if cmd == MSP_ACC_TRIM:
            return  # trim read sent at start-up; its reply is not used
        hint = REJECT_HINTS.get(cmd, "not supported by this firmware, or not allowed right now")
        print(f"Drone rejected MSP command {cmd} ({hint})")

    def _store_frame(self, cmd: int, payload: bytes) -> None:
        if cmd == MSP_ANALOG:
            self._fw_v2 = len(payload) >= 10  # MagisV2 sends 10 bytes, older Magis 7
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

        The reply's size tells which layout the board sends, and MSP_Protocol_Version is
        set from it: 1 for MagisV2 (10 bytes, BMS values), 0 for older Magis firmware
        (7 bytes, MultiWii layout).

        Returns:
            dict or None: None if no reply arrived. Keys depend on MSP_Protocol_Version.
        """
        data = self._get_frame(MSP_ANALOG, 7)
        if data is None:
            return None
        self.MSP_Protocol_Version = 1 if len(data) >= 10 else 0

        if self.MSP_Protocol_Version == 1:
            # u16 vBatComp (mV), mAmpRaw (mA), mAhDrawn, mAhRemain (mAh); u8 SoC (%), warning mode
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

    # The drone (MagisV2) sends its battery values ready to use: state of charge, mAh and
    # its warning come from its own BMS and are returned as sent. The helpers below read
    # one fresh MSP_ANALOG reply and return None without one, never a cached or made-up
    # value. Values the connected firmware does not send are None too.

    def _analog(self, key_v2: Optional[str], key_legacy: Optional[str] = None):
        """One value from a fresh MSP_ANALOG reply (MagisV2 key, older-Magis key), or None."""
        telemetry = self.get_analog_telemetry()
        if telemetry is None:
            return None
        key = key_v2 if telemetry['protocol_version'] == 1 else key_legacy
        return telemetry.get(key) if key else None

    def _clear_battery_cache(self) -> None:
        """Nothing from a previous link may show up as current battery data."""
        self.vBatComp = self.mAmpRaw = self.mAhDrawn = self.mAhRemain = 0
        self.soc_Fused = self.auto_LandMode = 0
        self.bytevbat = self.pMeterSum = self.rssi = self.amperage = 0

    def get_battery_voltage_compensated(self) -> Optional[float]:
        """Get compensated battery voltage (Volts), None if no data."""
        return self._analog('vBatComp', 'bytevbat')

    def get_current_mA(self) -> Optional[int]:
        """Get current draw (mA), None if no data."""
        return self._analog('mAmpRaw', 'amperage')

    def get_capacity_drawn_mAh(self) -> Optional[int]:
        """Get capacity consumed (mAh) as the drone reports it, None if no data (MagisV2 only)."""
        return self._analog('mAhDrawn')

    def get_capacity_remaining_mAh(self) -> Optional[int]:
        """Get capacity remaining (mAh) as the drone reports it, None if no data (MagisV2 only)."""
        return self._analog('mAhRemain')

    def get_state_of_charge(self) -> Optional[int]:
        """Get the drone's State of Charge (0-100%), None if no data (MagisV2 only)."""
        return self._analog('soc_Fused')

    def get_auto_land_status(self) -> Optional[int]:
        """
        Get the drone's battery warning code as sent: 0 OK, 1 WARNING, 2 CRITICAL.

        None if no data (MagisV2 only). Despite the name this is not an auto-land state:
        see get_battery_warning().
        """
        return self._analog('auto_LandMode')

    def get_rssi(self) -> Optional[int]:
        """Get signal strength RSSI, None if no data (older Magis only)."""
        return self._analog(None, 'rssi')

    def get_power_meter_sum(self) -> Optional[int]:
        """Get power meter sum, None if no data (older Magis only)."""
        return self._analog(None, 'pMeterSum')

    def get_battery_info(self) -> Optional[Dict[str, Any]]:
        """
        Get comprehensive battery information.
        
        Returns:
            dict or None: Dictionary containing battery telemetry, None if no data.
                  Structure depends on MSP_Protocol_Version.
        """
        telemetry = self.get_analog_telemetry()
        if not telemetry:
            return None
            
        if telemetry['protocol_version'] == 1:
            return {
                'voltage': f"{telemetry['vBatComp']:.2f}V",
                'current': f"{telemetry['mAmpRaw']}mA",
                'capacity_drawn': f"{telemetry['mAhDrawn']}mAh",
                'capacity_remaining': f"{telemetry['mAhRemain']}mAh",
                'state_of_charge': f"{telemetry['soc_Fused']}%",
                'auto_land_mode': telemetry['auto_LandMode'],
                'battery_warning': self._warning_text(telemetry['auto_LandMode']),
                'protocol': "V1 (Enhanced)"
            }
        return {
            'voltage': f"{telemetry['bytevbat']:.2f}V",
            'current': f"{telemetry.get('amperage', 0)}mA",
            'rssi': telemetry['rssi'],
            'power_sum': telemetry['pMeterSum'],
            'protocol': "Legacy"
        }

    @staticmethod
    def _level_from_soc(soc: int) -> str:
        if soc >= 70:
            return 'FULL'  # 4 bars - battery_full
        if soc >= 40:
            return 'TWO_BAR'  # 2 bars - battery_two_bar
        if soc >= 20:
            return 'ONE_BAR'  # 1 bar - battery_one_bar
        return 'EMPTY'  # Empty - no_battery

    @staticmethod
    def _warning_text(code: int) -> str:
        return BATTERY_WARNING_NAMES.get(code, f'UNKNOWN_{code}')

    def get_battery_level_status(self) -> Optional[str]:
        """
        Get battery level status based on the drone's state of charge.

        Returns:
            str or None: 'FULL', 'TWO_BAR', 'ONE_BAR', or 'EMPTY'; None if no data.
        """
        soc = self.get_state_of_charge()
        return None if soc is None else self._level_from_soc(soc)

    def get_battery_warning(self) -> Optional[str]:
        """
        Get the drone's own battery warning (MagisV2).

        The CG tilt firmware sends CRITICAL as WARNING while armed: it is then landing by
        itself, and the app only warns. No MSP reply tells "auto-landing" apart.

        Returns:
            str or None: 'OK', 'WARNING' or 'CRITICAL'; None if no data.
        """
        code = self.get_auto_land_status()
        return None if code is None else self._warning_text(code)

    def get_auto_land_status_text(self) -> Optional[str]:
        """
        Deprecated: use get_battery_warning(), which this now returns.

        It used to say 'NORMAL', 'LOW_BATTERY_WARNING' or 'AUTO_LANDING', but the drone's
        code is its battery warning (OK / WARNING / CRITICAL), not an auto-land state.
        """
        return self.get_battery_warning()

    def is_battery_critical(self) -> Optional[bool]:
        """
        Check if battery is in critical state (< 20% SOC).

        Returns:
            bool or None: True if critical, False otherwise, None if no data.
        """
        soc = self.get_state_of_charge()
        return None if soc is None else soc < 20

    def is_battery_low(self) -> Optional[bool]:
        """
        Check if battery is low (< 40% SOC).

        Returns:
            bool or None: True if low, False otherwise, None if no data.
        """
        soc = self.get_state_of_charge()
        return None if soc is None else soc < 40

    def should_auto_land(self) -> Optional[bool]:
        """
        Check if the drone reports a low or critical battery: land now.

        The firmware does not report whether its own auto-land is running (see
        get_battery_warning()), so this is True on its WARNING or CRITICAL.

        Returns:
            bool or None: True if the drone warns about its battery, None if no data.
        """
        code = self.get_auto_land_status()
        return None if code is None else code >= 1

    def get_battery_status_summary(self) -> Optional[Dict[str, Any]]:
        """
        Get comprehensive battery status summary matching mobile app display.

        Built from one MSP_ANALOG reply, so all values belong together.

        Returns:
            dict or None: Detailed status including icon, warnings, and flags; None if no
            data.
        """
        telemetry = self.get_analog_telemetry()
        if telemetry is None:
            return None
        if telemetry['protocol_version'] != 1:
            return {
                'error': 'Enhanced telemetry only available in Protocol V1',
                'voltage': telemetry['bytevbat'],
                'protocol': telemetry['protocol_version']
            }

        soc = telemetry['soc_Fused']
        code = telemetry['auto_LandMode']
        battery_level = self._level_from_soc(soc)
        warning = self._warning_text(code)

        # Determine status icon/emoji
        icon = '🔋' if battery_level in ('FULL', 'TWO_BAR') else '🪫'

        # Determine warning message
        warnings = []
        if warning == 'CRITICAL':
            warnings.append('⚠️ BATTERY CRITICAL - Land Now!')
        elif warning == 'WARNING':
            warnings.append('⚠️ LOW BATTERY - Please Land')

        if soc < 20:
            warnings.append('🔴 CRITICAL - Land Immediately!')
        elif soc < 40:
            warnings.append('🟡 LOW - Consider Landing')

        return {
            'icon': icon,
            'voltage': f"{telemetry['vBatComp']:.2f}V",
            'soc': f"{soc}%",
            'capacity_remaining': f"{telemetry['mAhRemain']}mAh",
            'battery_level': battery_level,
            'battery_warning': warning,
            'auto_land_status': warning,  # old key, same value as battery_warning
            'warnings': warnings,
            'is_critical': soc < 20,
            'is_low': soc < 40,
            'should_auto_land': code >= 1
        }

    # ── Battery capacity stored in the drone (MagisV2) ──────────────────────

    def get_battery_capacity(self) -> Optional[int]:
        """
        The battery capacity stored in the drone, in mAh (MagisV2).

        The drone computes its state of charge and mAh remaining from it.

        Returns:
            int or None: Capacity in mAh, None if no reply.
        """
        if not self._requires_magis_v2("get_battery_capacity"):
            return None
        data = self._request(MSP_VOLTAGE_METER_CONFIG, 5)
        return None if data is None else int.from_bytes(data[3:5], 'little')

    def set_battery_capacity(self, capacity_mah: int) -> bool:
        """
        Store a new battery capacity in the drone, in mAh (MagisV2), e.g. 1200 for a Pluto
        Guru pack.

        The drone uses a new capacity only after a restart, so saving it stops the flight
        controller: it blinks its LEDs green and blue until you switch it off and on. Call
        it on the ground with the drone disarmed. The library checks that the drone
        stopped, disconnects, and tells you to power-cycle it. The battery voltages stored
        with the capacity are kept, and nothing is written if the capacity is already set.

        Args:
            capacity_mah (int): 100-5000 mAh.

        Returns:
            bool: True if the new capacity was saved (or was already set).
        """
        if not self._in_range("battery capacity", capacity_mah, BATTERY_CAPACITY_MIN, BATTERY_CAPACITY_MAX):
            return False
        if not self._requires_magis_v2("set_battery_capacity") or self._refuse_if_armed("set_battery_capacity"):
            return False
        data = self._request(MSP_VOLTAGE_METER_CONFIG, 5)
        if data is None:
            print("set_battery_capacity: no reply from the drone, nothing changed")
            return False
        v_max, v_min, v_warning = data[0], data[1], data[2]
        current = int.from_bytes(data[3:5], 'little')
        if current == capacity_mah:
            # The drone stops after any capacity write, even an unchanged one: don't write.
            print(f"Battery capacity is already {current} mAh: nothing to change")
            return True

        # The write order is max, warning, min (the read order is max, min, warning)
        payload = [v_max, v_warning, v_min] + list(capacity_mah.to_bytes(2, 'little'))
        sent_at = time.monotonic()
        if not (self.send_request_msp(self.create_packet_msp_u8(MSP_SET_VOLTAGE_METER_CONFIG, payload))
                and self.send_request_msp(self.create_packet_msp(MSP_EEPROM_WRITE, []))):
            return False

        # After saving, the drone stops answering. Wait up to 4 s for 1 s of silence.
        stopped = False
        while time.monotonic() - sent_at < 4.0 and self.connected:
            time.sleep(0.1)
            with self._frames_cv:
                last = max((t for _, t in self._frames.values()), default=0.0)
            if time.monotonic() - max(last, sent_at) >= 1.0:
                stopped = True
                break
        if not stopped and self.connected:
            print("set_battery_capacity: the drone kept answering, so the new capacity may not be "
                  "saved. Check it with get_battery_capacity().")
            return False
        print(f"Battery capacity saved: {capacity_mah} mAh (was {current} mAh). The drone is "
              "blinking its LEDs: switch it off and on, then connect() again.")
        self.disconnect()
        return True
