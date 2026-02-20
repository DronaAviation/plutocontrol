import socket
from threading import Thread, Lock, Event
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

class pluto:
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
        self._stop_event = Event()  # Signals thread to stop
        self.thread = None
        self.client = None
        
        # Developer Mode (Serial Debug)
        self.dev_mode_enabled = False
        self.debug_buffer = bytearray()



    def start_write_function(self):
        """Start the background communication thread"""
        if self.thread is None or not self.thread.is_alive():
            self._stop_event.clear()
            self.thread = Thread(target=self.write_function, daemon=True)
            self.thread.start()
        
    def connect(self) -> None:
        """
        Establish a TCP connection to the drone and start the communication thread.
        
        Default IP: 192.168.4.1
        Default Port: 23
        
        Raises:
            socket.timeout: If connection fails to establish within 5 seconds.
        """
        if not self.connected:
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client.settimeout(5.0)  # 5 second timeout for connection
            try:
                self.client.connect((self.TCP_IP, self.TCP_PORT))
                self.connected = True
                print(f"Connected to drone at {self.TCP_IP}:{self.TCP_PORT}")
                self.start_write_function()
            except socket.timeout:
                print(f"Connection timeout: Unable to connect to {self.TCP_IP}:{self.TCP_PORT}")
                self.connected = False
            except Exception as e:
                print(f"Error connecting to server: {e}")
                self.connected = False

    def disconnect(self) -> None:
        """
        Disconnect from the drone and stop the background communication thread.
        Cleanly closes the socket and releases resources.
        """
        if self.connected:
            self.connected = False
            self._stop_event.set()  # Signal thread to stop
            
            # Wait for thread to finish (with timeout)
            if self.thread and self.thread.is_alive():
                self.thread.join(timeout=1.0)
            
            # Close socket
            if self.client:
                try:
                    self.client.close()
                except:
                    pass
            
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
            self.command_type = self.LAND2

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

    def trim(self, roll: int, pitch: int) -> None:
        """
        Set ACC Trim values.
        
        Args:
            roll (int): Roll trim value.
            pitch (int): Pitch trim value.
        """
        roll = max(min(roll, TRIM_MAX), TRIM_MIN)
        pitch = max(min(pitch, TRIM_MAX), TRIM_MIN)
        
        # Pass as list of 16-bit integers [roll, pitch]
        self.send_request_msp(self.create_packet_msp(MSP_ACC_TRIM, [roll, pitch]))
            
    def send_request_msp(self, data):
       if self.connected:
           self.client.send(bytes.fromhex(data))
       else:
           if not hasattr(self, 'connection_error_logged'):
               print("Error: Not connected to server")
               self.connection_error_logged = True

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

    def send_request_msp_set_raw_rc(self, channels):
        self.send_request_msp(self.create_packet_msp(MSP_SET_RAW_RC, channels))

    def send_request_msp_set_command(self, command_type):
        self.send_request_msp(self.create_packet_msp(MSP_SET_COMMAND, [command_type]))

    def send_request_msp_get_debug(self, requests):
        for i in range(len(requests)):
            self.send_request_msp(self.create_packet_msp(requests[i], []))

    def send_request_msp_set_acc_trim(self, trim_roll, trim_pitch):
        self.send_request_msp(self.create_packet_msp(MSP_ACC_TRIM, [trim_roll, trim_pitch]))

    def send_request_msp_acc_trim(self):
        if self.connected:
            self.send_request_msp(self.create_packet_msp(MSP_ACC_TRIM, []))
        else:
            print("Error: Not connected to server")

    def send_request_msp_eeprom_write(self):
        self.send_request_msp(self.create_packet_msp(MSP_EEPROM_WRITE, []))

    def send_request_msp_set_motor(self, motor_speeds):
        payload = []
        for speed in motor_speeds:
            payload.append(speed & 0xFF)
            payload.append((speed >> 8) & 0xFF)
        self.send_request_msp(self.create_packet_msp(MSP_SET_MOTOR, payload))

    def write_function(self):
        """Background thread that continuously sends commands to the drone"""
        requests = [MSP_RC, MSP_ATTITUDE, MSP_RAW_IMU, MSP_ALTITUDE, MSP_ANALOG]
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
            self.send_request_msp_get_debug(requests)
            if cmd_to_send != self.NONE_COMMAND:
                self.send_request_msp_set_command(cmd_to_send)

            time.sleep(0.022)  # ~45Hz update rate

    def get_height(self) -> float:
        """
        Get the current height of the drone.
        
        Returns:
            float: Height in cm (or other unit depending on sensor).
        """
        data = []
        self.create_packet_msp(MSP_ALTITUDE, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 109:
                i += 1
            if i + 3 < len(data):
                height = self.read16(data[i + 1:i + 3])
                print(f"height: {height} cm")
                return height
        return 0.0

    def get_vario(self):
        data = []
        self.create_packet_msp(MSP_ALTITUDE, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 109:
                i += 1
            if i + 7 < len(data):
                return self.read16(data[i + 5:i + 7])

    def get_roll(self) -> float:
        """
        Get the current Roll angle.
        
        Returns:
            float: Roll angle in degrees.
        """
        data = []
        self.create_packet_msp(MSP_ATTITUDE, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 108:
                i += 1
            if i + 3 < len(data):
                roll = self.read16(data[i + 1:i + 3]) / 10
                print(f"roll: {roll} degrees")
                return roll
        return 0.0

    def get_pitch(self) -> float:
        """
        Get the current Pitch angle.
        
        Returns:
            float: Pitch angle in degrees.
        """
        data = []
        self.create_packet_msp(MSP_ATTITUDE, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 108:
                i += 1
            if i + 5 < len(data):
                pitch = self.read16(data[i + 3:i + 5]) / 10
                print(f"pitch: {pitch} degrees")
                return pitch
        return 0.0

    def get_yaw(self) -> float:
        """
        Get the current Yaw angle.
        
        Returns:
            float: Yaw angle in degrees.
        """
        data = []
        self.create_packet_msp(MSP_ATTITUDE, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 108:
                i += 1
            if i + 7 < len(data):
                yaw = self.read16(data[i + 5:i + 7])
                print(f"yaw: {yaw} degrees")
                return float(yaw)
        return 0.0

    def get_acc_x(self) -> int:
        """Get raw Accelerometer X value (unsigned 16-bit)."""
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 3 < len(data):
                return self.read16(data[i + 1:i + 3])
        return 0

    def get_acc_y(self) -> int:
        """Get raw Accelerometer Y value (unsigned 16-bit)."""
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 5 < len(data):
                return self.read16(data[i + 3:i + 5])
        return 0

    def get_acc_z(self) -> int:
        """Get raw Accelerometer Z value (unsigned 16-bit)."""
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 7 < len(data):
                return self.read16(data[i + 5:i + 7])
        return 0

    def get_gyro_x(self) -> int:
        """Get raw Gyroscope X value (unsigned 16-bit)."""
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 9 < len(data):
                return self.read16(data[i + 7:i + 9])
        return 0

    def get_gyro_y(self) -> int:
        """Get raw Gyroscope Y value (unsigned 16-bit)."""
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 11 < len(data):
                return self.read16(data[i + 9:i + 11])
        return 0

    def get_gyro_z(self) -> int:
        """Get raw Gyroscope Z value (unsigned 16-bit)."""
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 13 < len(data):
                return self.read16(data[i + 11:i + 13])
        return 0

    def get_mag_x(self) -> int:
        """Get raw Magnetometer X value (unsigned 16-bit)."""
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 15 < len(data):
                return self.read16(data[i + 13:i + 15])
        return 0

    def get_mag_y(self) -> int:
        """Get raw Magnetometer Y value (unsigned 16-bit)."""
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 17 < len(data):
                return self.read16(data[i + 15:i + 17])
        return 0

    def get_mag_z(self) -> int:
        """Get raw Magnetometer Z value (unsigned 16-bit)."""
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 19 < len(data):
                return self.read16(data[i + 17:i + 19])
        return 0

    def calibrate_acceleration(self):
        self.create_packet_msp(MSP_ACC_CALIBRATION, [])
        time.sleep(5)  # Adjust sleep duration based on calibration time
        acc_x = self.get_acc_x()
        acc_y = self.get_acc_y()
        acc_z = self.get_acc_z()
        print("Calibrated Accelerometer Values:")
        print(f"X-Axis: {acc_x}, Y-Axis: {acc_y}, Z-Axis: {acc_z}")
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
            float: Battery voltage in Volts.
        """
        data = []
        self.create_packet_msp(MSP_ANALOG, data) 
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 110:
                i += 1
            if i + 1 < len(data):
                return self.read8(data[i + 1]) / 10.0
        return 0.0

    def get_battery_percentage(self) -> float:
        """
        Get the current battery percentage (estimated).
        
        Returns:
            float: Battery percentage (0.0 to 100.0).
        """
        data = []
        self.create_packet_msp(MSP_ANALOG, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0 
            while i < len(data) and data[i] != 110:
                i += 1
            if i+1 < len(data):
                voltage = self.read8(data[i+1]) / 10.0
                return ((voltage - 3.7) / (4.2 - 3.7)) * 100
        return 0.0
            
    # Removed enable_raw_log function as per request for simplicity

    def receive_packet(self):
        try:
            # Read from socket
            data = self.client.recv(4096)
            if not data:
                return b''
            
            # Check for debug messages ($MD) if dev mode is on
            # We scan the received chunk for the debug header or raw text
            if self.dev_mode_enabled:
                self.check_and_print_debug_msg(data)
                
            return data
        except socket.timeout:
            return b''
        except Exception as e:
            # Only print error if connected to avoid spam on disconnect
            if self.connected:
                print(f"Receive error: {e}")
            return b''

    def check_and_print_debug_msg(self, data: bytes) -> None:
        """
        Internal method to parse and print debug messages/strings from the drone.
        Only active when dev_mode_enabled is True.
        
        Handles:
        1. MSP Debug Packets ($D...)
        2. Raw Serial Text (newline terminated)
        """
        if not self.dev_mode_enabled or not data:
            return
        self.debug_buffer.extend(data)
        
        # Limit buffer to prevent infinite growth
        if len(self.debug_buffer) > 4096:
             self.debug_buffer = self.debug_buffer[-2048:]
            
        while True:
            # Check for potential MSP Debug Header ($D)
            idx_msp = self.debug_buffer.find(b'$D')
            
            # Check for potential Newlines (Raw Text)
            idx_newline = -1
            try:
                # Find first newline
                for i in range(len(self.debug_buffer)):
                    if self.debug_buffer[i] == 10: # \n
                        idx_newline = i
                        break
            except: 
                pass

            # Decision Logic: Process whichever comes first, or process text if no MSP
            
            # CASE 1: Found MSP Header
            if idx_msp != -1:
                # If newline comes BEFORE MSP header, process text first (it's garbage/raw)
                if idx_newline != -1 and idx_newline < idx_msp:
                    line_data = self.debug_buffer[:idx_newline+1]
                    try:
                        line = line_data.decode('utf-8', errors='ignore').strip()
                        if line and len(line) > 1 and not line.startswith('$'):
                            print(f"[SERIAL] {line}")
                    except: pass
                    self.debug_buffer = self.debug_buffer[idx_newline+1:]
                    continue
                
                # Process MSP Packet
                if idx_msp + 3 <= len(self.debug_buffer):
                    payload_size = self.debug_buffer[idx_msp+2]
                    total_packet_size = 4 + payload_size
                    
                    if idx_msp + total_packet_size <= len(self.debug_buffer):
                         # Extract payload
                        payload = self.debug_buffer[idx_msp+3 : idx_msp+3+payload_size]
                        try:
                            debug_msg = payload.decode('utf-8', errors='ignore').strip()
                            if debug_msg and not debug_msg.startswith('~'):
                                timestamp = time.strftime("%H:%M:%S")
                                print(f"[DEBUG {timestamp}] {debug_msg}")
                        except:
                            pass
                        
                        # Remove packet
                        self.debug_buffer = self.debug_buffer[idx_msp + total_packet_size:]
                        continue
                    else:
                        # Incomplete MSP packet, wait for more data
                        break
                else:
                    # Incomplete header, wait for more data
                    break
            
            # CASE 2: No MSP Header, but found Newline (Raw Text)
            elif idx_newline != -1:
                line_data = self.debug_buffer[:idx_newline+1]
                try:
                    line = line_data.decode('utf-8', errors='ignore').strip()
                    # Filter out purely binary-looking lines or extremely short ones unless clearly text
                    if line and len(line) > 1:
                        # Ensure it's not a fragment of another MSP command
                        if not line.startswith('$M'): 
                             print(f"[SERIAL] {line}")
                except: pass
                self.debug_buffer = self.debug_buffer[idx_newline+1:]
                continue
                
            # No relevant data found
            break

    def read16(self, arr):
        if (arr[1] & 0x80) == 0:
            return (arr[1] << 8) + (arr[0] & 0xff)
        else:
            return (-65535 + (arr[1] << 8) + (arr[0] & 0xff))
    
    def read8(self, value):
        """Read single byte value"""
        return value & 0xFF

    # Enhanced Battery Telemetry Methods
    
    def get_analog_telemetry(self):
        """
        Get comprehensive analog telemetry data including battery info
        Supports both Protocol Version 1 and other versions
        Returns a dictionary with all available telemetry
        """
        data = self.receive_packet()
        i = 0
        
        # Find MSP_ANALOG response (command 110)
        while i < len(data) and data[i] != MSP_ANALOG:
            i += 1
        
        if i >= len(data):
            return None
        
        telemetry = {}
        
        if self.MSP_Protocol_Version == 1:
            # Protocol Version 1: Enhanced telemetry
            if i + 11 < len(data):
                self.vBatComp = self.read16(data[i + 1:i + 3])
                self.mAmpRaw = self.read16(data[i + 3:i + 5])
                self.mAhDrawn = self.read16(data[i + 5:i + 7])
                self.mAhRemain = self.read16(data[i + 7:i + 9])
                self.soc_Fused = self.read8(data[i + 9])
                self.auto_LandMode = self.read8(data[i + 10])
                
                telemetry = {
                    'vBatComp': self.vBatComp / 1000.0,  # Convert to volts (divide by 1000)
                    'mAmpRaw': self.mAmpRaw,
                    'mAhDrawn': self.mAhDrawn,
                    'mAhRemain': self.mAhRemain,
                    'soc_Fused': self.soc_Fused,
                    'auto_LandMode': self.auto_LandMode,
                    'protocol_version': 1
                }
        else:
            # Other versions: Basic telemetry
            if i + 7 < len(data):
                self.bytevbat = self.read8(data[i + 1])
                self.pMeterSum = self.read16(data[i + 2:i + 4])
                self.rssi = self.read16(data[i + 4:i + 6])
                self.amperage = self.read16(data[i + 6:i + 8])
                
                telemetry = {
                    'bytevbat': self.bytevbat / 10.0,  # Convert to volts
                    'pMeterSum': self.pMeterSum,
                    'rssi': self.rssi,
                    'amperage': self.amperage,
                    'protocol_version': self.MSP_Protocol_Version
                }
        
        return telemetry
    
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
