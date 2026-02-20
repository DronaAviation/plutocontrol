# plutocontrol

plutocontrol is a Python library for controlling Pluto drones. This library provides various methods to interact with the drone, including connecting, controlling movements, and accessing sensor data.

## Installation

```bash
pip install plutocontrol
```

## Usage

After installing the package, you can import and use the `Pluto` class in your Python scripts.

### Configuration

By default, the package connects to the drone at IP `192.168.4.1` on port `23`. You can customize these values when creating a Pluto instance:

```python
from plutocontrol import Pluto

# Use default IP (192.168.4.1) and port (23)
pluto = Pluto()

# Use custom IP address when in ST mode
pluto = Pluto(ip='192.168.1.100')

```

### Example Usage

#### Example 1: Basic Usage (Default IP)

```python
from plutocontrol import Pluto

# Create an instance of the Pluto class with default IP (192.168.4.1:23)
pluto = Pluto()

# Connect to the drone
pluto.connect()

# Arm the drone
pluto.arm()

# Disarm the drone
pluto.disarm()

# Disconnect from the drone
pluto.disconnect()
```

#### Example 2: Custom IP Address when drone is connected to router with ST mode

```python
from plutocontrol import Pluto
import time

# Create an instance with custom IP
pluto = Pluto(ip='192.168.1.100')

# Connect to the drone
pluto.connect()
time.sleep(1)

# Arm and take off
pluto.arm()
time.sleep(2)
pluto.take_off()
time.sleep(5)

# Fly forward
pluto.forward()
time.sleep(2)
pluto.reset()

# Land and disconnect
pluto.land()
time.sleep(3)
pluto.disconnect()
```



## Class and Methods

### Pluto Class


#### `Connection`

Commands to connect/ disconnect to the drone server.

```python
#Connects from the drone server.
pluto.connect()

#Disconnects from the drone server.
pluto.disconnect()
```

#### `Camera module`
Sets the IP and port for the camera connection. should be initialized before pluto.connect().

```python
pluto.cam()
```

#### `Arm and Disarm Commands`

```python
#Arms the drone, setting it to a ready state.
pluto.arm()

#Disarms the drone, stopping all motors.
pluto.disarm()
```

#### `Pitch Commands`

```python
#Sets the drone to move forward.
pluto.forward()

#Sets the drone to move backward.
pluto.backward()
```

#### `Roll Commands`

```python
#Sets the drone to move left (roll).
pluto.left()

#Sets the drone to move right (roll).
pluto.right()
```

#### `Yaw Commands`

```python
#Sets the drone to yaw right.
pluto.right_yaw()

#Sets the drone to yaw left.
pluto.left_yaw()
```

#### `Throttle Commands`

Increase/ Decrease the drone's height.

```Python
#Increases the drone's height.
pluto.increase_height()

#Decreases the drone's height.
pluto.decrease_height()
```

#### `Takeoff and Land`

```Python
#Arms the drone and prepares it for takeoff.
pluto.take_off()

#Commands the drone to land.
pluto.land()
```

#### `Developer Mode`
Toggle Developer Mode

```Python
#Turns the Developer mode ON
pluto.devOn()

#Turns the Developer mode OFF
pluto.devOff()
```

#### `motor_speed(motor_index, speed)`
Sets the speed of a specific motor (motor index from 0 to 3).

```Python
pluto.motor_speed(0, 1500)
```

#### `Get MSP_ALTITUDE Values`

```python
#Returns the height of the drone from the sensors.
height = pluto.get_height()

#Returns the rate of change of altitude from the sensors.
vario = pluto.get_vario()
```

#### `Get MSP_ALTITUDE Values`

```python
#Returns the roll value from the drone.
roll = pluto.get_roll()

#Returns the pitch value from the drone.
pitch = pluto.get_pitch()

#Returns the yaw value from the drone.
yaw = pluto.get_yaw()
```

#### `Get MSP_RAW_IMU Values`

##### `Accelerometer`
Returns the accelerometer value for the x,y,z - axis.

```python
#Returns the accelerometer value for the x-axis.
acc_x = pluto.get_acc_x()

#Returns the accelerometer value for the y-axis.
acc_y = pluto.get_acc_y()

#Returns the accelerometer value for the z-axis.
acc_z = pluto.get_acc_z()
```

#### `Gyroscope`
Returns the Gyroscope value for the x,y,z - axis.

```python
#Returns the Gyroscope value for the x-axis.
gyro_x = pluto.get_gyro_x()

#Returns the Gyroscope value for the y-axis.
gyro_y = pluto.get_gyro_y()

#Returns the Gyroscope value for the z-axis.
gyro_z = pluto.get_gyro_z()
```

#### `Magnetometer`
Returns the Magntometer value for the x,y,z - axis.

```python
#Returns the Magnetometer value for the x-axis.
mag_x = pluto.get_mag_x()

#Returns the Magnetometer value for the y-axis.
mag_y = pluto.get_mag_y()

#Returns the Magnetometer value for the z-axis.
mag_z = pluto.get_mag_z()
```

#### `Calibration Commands`

```python
#Calibrates the accelerometer.
pluto.calibrate_acceleration()

#Calibrates the magnetometer.
pluto.calibrate_magnetometer()
```

#### `Get MSP_Analog Values`

```python
#Returns the battery value in volts from the drone.
battery = pluto.get_battery()

#Returns the battery percentage from the drone.
battery_percentage = pluto.get_battery_percentage()
```

## Enhanced Battery Telemetry (Mobile App Features)

The package now supports enhanced battery telemetry matching the mobile app functionality, with support for multiple MSP protocol versions.

### Protocol Version Detection

The drone's MSP protocol version is automatically detected:

```python
# Check protocol version
print(f"Protocol Version: {pluto.MSP_Protocol_Version}")
```

### Comprehensive Battery Information

Get all battery information in one call:

```python
#Get comprehensive battery info (all parameters)
battery_info = pluto.get_battery_info()

# Protocol V1 returns:
# {
#     'voltage': '11.25V',
#     'current': '1250mA',
#     'capacity_drawn': '450mAh',
#     'capacity_remaining': '1350mAh',
#     'state_of_charge': '75%',
#     'auto_land_mode': 0,
#     'protocol': 'V1 (Enhanced)'
#     'protocol': 'V1 (Enhanced)'
# }
```

### Battery Helper Methods
Convenient methods to check battery status without parsing raw values.

```python
# Get status string ('FULL', 'TWO_BAR', 'ONE_BAR', 'EMPTY')
status = pluto.get_battery_level_status()

# Check for critical battery (< 20%)
if pluto.is_battery_critical():
    print("LAND NOW!")

# Check if auto-landing is triggered
if pluto.should_auto_land():
    print("Drone is auto-landing due to low battery")

# Get a full summary dict (for UI/Display)
summary = pluto.get_battery_status_summary()
print(f"Icon: {summary['icon']} | Voltage: {summary['voltage']}")
```

### Individual Telemetry Parameters

#### Enhanced Telemetry on Primus X2 / V5 boards

```python
#Get compensated battery voltage in volts
voltage = pluto.get_battery_voltage_compensated()

#Get current draw in milliamps
current = pluto.get_current_mA()

#Get capacity consumed in mAh
capacity_drawn = pluto.get_capacity_drawn_mAh()

#Get remaining capacity in mAh
capacity_remaining = pluto.get_capacity_remaining_mAh()

#Get state of charge (0-100%)
soc = pluto.get_state_of_charge()

#Get auto-land mode status
auto_land = pluto.get_auto_land_status()
```

#### Basic Telemetry on Pluto  / V4 boards

```python
#Get battery voltage
voltage = pluto.get_battery_voltage_compensated()

#Get current draw
current = pluto.get_current_mA()

#Get signal strength (RSSI)
rssi = pluto.get_rssi()

#Get power meter sum
power_sum = pluto.get_power_meter_sum()
```

### Telemetry Data Structure

**Protocol Version 1:**
- `vBatComp`: Battery voltage compensated (in centiv olts, divide by 100 for volts)
- `mAmpRaw`: Current draw in milliamps
- `mAhDrawn`: Capacity consumed in milliamp-hours
- `mAhRemain`: Remaining capacity in milliamp-hours
- `soc_Fused`: State of charge percentage (0-100)
- `auto_LandMode`: Auto-land mode status (0=off, 1=on)

**Other Versions:**
- `bytevbat`: Battery voltage in decivolts (divide by 10 for volts)
- `pMeterSum`: Power meter sum
- `rssi`: Signal strength indicator
- `amperage`: Current draw in milliamps


## Wifi Configuration (Telnet / AT Commands)

To control multiple drones in the same network or to change the drone's Wifi settings (Stations Mode - ST), you can configure the Pluto drone using Telnet commands.

### Prerequisites
- A Telnet client (e.g., [Putty](https://www.putty.org/) for Windows, or terminal `telnet` for Mac/Linux).
- Connect your computer to Pluto's Wifi (Default SSID: `Pluto_XXXX`, Password: `dronaaviation`).

### Accessing Configuration
1. Connect to Pluto's Wifi.
2. Open your Telnet client.
3. Connect to **IP: 192.168.4.1** on **Port: 23**.

### Testing Connection
Type the following command:
```text
+++AT
```
Response should be `OK`.

### Common AT Commands

| Command | Description |
|---------|-------------|
| `+++AT` | Test connection. Response: `OK` |
| `+++AT RESET` | Soft reset the ESP (Wifi Module) |
| `+++AT MODE` | Print current mode settings |
| `+++AT MODE <mode>` | Set Wifi mode: <br> `1` : **STA** (Station Mode - Connect to Router) <br> `2` : **AP** (Access Point - Default) <br> `3` : **Both** |
| `+++AT STA` | Print current Station SSID and Password |
| `+++AT STA <SSID> <password>` | Set the Router SSID and Password for Station Mode |

### Example: Setting up Station Mode (Connecting Pluto to Router)

To connect your Pluto drone to your home router so you can control it alongside other devices:

1. Connect to Pluto via Telnet.
2. Set the Router credentials:
   ```text
   +++AT STA MyRouterName MyRouterPassword
   ```
3. Change mode to Station (or Both):
   ```text
   +++AT MODE 3
   ```
4. Reset the drone/modules for changes to take effect.


## Debugging and Developer Mode

If you are developing custom features or want to see what's happening inside the drone, you can enable **Serial Debug Mode**. This prints debug messages from the drone directly to your terminal.

```python
# Enable Developer Mode (Serial Debug Output)
pluto.devOn()

# ... perform actions ...

# Disable Developer Mode
pluto.devOff()
```

**Note:** The library automatically detects and prints both:
- **MSP Debug Packets**: Structural debug data sent by the flight controller.
- **Raw Serial Strings**: `printf` style logs used in custom firmware.


## Beginner Project Ideas

Here are some simple projects to get started with `plutocontrol`.

### Project 1: The "Hello World" Flight
A simple script to take off, hover for 3 seconds, and land.

```python
from plutocontrol import Pluto
import time

my_pluto = Pluto()
my_pluto.connect()

print("Arming...")
my_pluto.arm()
time.sleep(2)

print("Taking Off!")
my_pluto.take_off()
time.sleep(3) # Hover for 3 seconds

print("Landing...")
my_pluto.land()
time.sleep(2)

my_pluto.disarm()
my_pluto.disconnect()
```

### Project 2: Keyboard Controller
Control your drone using your computer keyboard!

```python
from plutocontrol import Pluto
import threading
import keyboard  # pip install keyboard

pluto = Pluto()
pluto.connect()
pluto.arm()

def control_loop():
    while True:
        if keyboard.is_pressed('w'): pluto.forward()
        elif keyboard.is_pressed('s'): pluto.backward()
        elif keyboard.is_pressed('a'): pluto.left()
        elif keyboard.is_pressed('d'): pluto.right()
        elif keyboard.is_pressed('space'): pluto.take_off()
        elif keyboard.is_pressed('x'): pluto.land()
        elif keyboard.is_pressed('q'): break # Quit
        else: 
            # Reset RC to neutral if keys released
            pluto.rcPitch = 1500
            pluto.rcRoll = 1500
            
control_loop()
pluto.disarm()
pluto.disconnect()
```

