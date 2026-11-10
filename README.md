# plutocontrol

plutocontrol is a Python library for controlling Pluto drones. This library provides various methods to interact with the drone, including connecting, controlling movements, and accessing sensor data.

## Installation

```bash
pip install plutocontrol
```

Needs Python 3.8 or newer.

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

### Try Every Function

`examples/api_test_menu.py` lists every plutocontrol function by number: type a number to
run it, and it asks for any values it needs. `t` runs every read-only function once as a
quick self-test, `x` or Ctrl+C disarms at once, `q` quits.

```bash
python examples/api_test_menu.py               # asks for the drone's IP
python examples/api_test_menu.py 192.168.4.1   # IP given
```

Try it on the ground with the propellers off first: movement and height commands make the
drone fly.

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
#Connects to the drone server. Returns True if connected, False if it failed.
if not pluto.connect():
    raise SystemExit("Could not connect to the drone")

#Disconnects from the drone server.
pluto.disconnect()
```

If the drone has been answering and then goes silent for 3 seconds (for example out of
Wi-Fi range), the library treats the link as lost: `pluto.connected` becomes `False`, it
prints why, and `pluto.connect()` reconnects.

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

#### `Movement Commands`

Each command holds its stick at `1500 +/- strength` until `reset()` or another command
changes it. `strength` is optional: an integer from 0 to 500. On MagisV2 firmware the
default is 200 in every direction, so opposite moves are equally strong. Drones with older
Magis firmware keep their original defaults (shown in brackets below).

```python
#Pitch: move forward / backward (pitch 1700 / 1300) [older Magis: 1600 / 1300].
pluto.forward()
pluto.backward()

#Roll: move left / right (roll 1300 / 1700) [older Magis: 1200 / 1600].
pluto.left()
pluto.right()

#Yaw: rotate left / right (yaw 1300 / 1700) [older Magis: 1300 / 1600].
pluto.left_yaw()
pluto.right_yaw()

#Gentler or stronger moves, e.g. in a small room or outdoors.
pluto.forward(100)   # pitch 1600
pluto.left(300)      # roll 1200

#Back to centre (roll, pitch, yaw and throttle 1500).
pluto.reset()
```

On MagisV2 firmware the library flies in headfree mode (AUX1 = 1500): forward, backward,
left and right stay relative to the drone's starting heading, not to where its nose points
after a yaw.

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
#Arms the drone and takes off. On MagisV2 it climbs to about 120 cm and holds that height.
pluto.take_off()

#Commands the drone to land.
pluto.land()
```

On MagisV2 firmware `take_off()` centres the sticks (throttle 1500): the firmware only
takes off with the throttle centred. Older Magis firmware gets throttle 1800, as before.

#### `Developer Mode`
Toggle Developer Mode

```Python
#Turns the Developer mode ON
pluto.devOn()

#Turns the Developer mode OFF
pluto.devOff()
```

#### `motor_speed(motor_index, speed)`
Motor test: spins one motor while the drone is **disarmed**. `motor_index` is an integer
from 0 to 3 and `speed` an integer from 1000 to 2000. Anything else is refused with a
message and nothing is sent.

The motor keeps spinning until `motor_speed(index, 1000)`, `arm()`, `take_off()` or
`disconnect()`. Keep hands clear. On MagisV2 firmware it is refused while armed: the drone
would keep the value and spin the motor as soon as it disarms.

```Python
pluto.motor_speed(0, 1500)   # spin motor 0
pluto.motor_speed(0, 1000)   # stop it
```

#### `Telemetry`

The getters below return the latest value from the drone, or `None` when there is no
current data (not connected, or nothing from the drone for about a second). Check for
`None` before using a value:

```python
height = pluto.get_height()
if height is not None and height > 100:
    pluto.decrease_height()
```

#### `Get MSP_ALTITUDE Values`

```python
#Returns the height of the drone from the sensors.
height = pluto.get_height()

#Returns the rate of change of altitude from the sensors.
vario = pluto.get_vario()
```

#### `Get MSP_ATTITUDE Values`

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

Place the drone on a flat, level surface and keep it still for a few seconds while it
calibrates. The drone ignores calibration while armed (without an error), so on MagisV2
firmware these are refused while armed: call `disarm()` first.

```python
#Calibrates the accelerometer.
pluto.calibrate_acceleration()

#Calibrates the magnetometer.
pluto.calibrate_magnetometer()
```

#### `Flight Status` (MagisV2)

The drone's own status, as the Pluto app shows it. Use it to find out why `arm()` or
`take_off()` did nothing. Returns `None` if there is no reply.

```python
status = pluto.get_flight_status()
# 'ACCEL_GYRO_CALIBRATION', 'MAG_CALIBRATION', 'CRASH', 'LOW_BATTERY',
# 'LOW_BATTERY_IN_FLIGHT', 'SIGNAL_LOSS', 'NOT_OK_TO_ARM', 'OK_TO_ARM' or 'ARMED'

#True / False, or None when the drone reports a more important state
#(crash, low battery, signal loss) that hides whether it is armed.
armed = pluto.is_armed()
```

The drone reports one state at a time, the most important first: calibrating, crash, low
battery, signal loss, not OK to arm, OK to arm, armed.

#### `Height Ceiling` (MagisV2)

A safety ceiling for indoor flying: in altitude hold the drone does not climb above it.
The drone keeps it until it reboots.

```python
pluto.set_max_height(100)     # stay below 1 m
ceiling = pluto.get_max_height()   # cm, -1 if there is none, None if no reply
pluto.set_max_height(None)    # remove the ceiling
```

#### `set_height(cm)`: hold a height

Sets the height the drone holds, in cm (20-10000). The drone climbs or descends to it
smoothly. Call it after take-off (take-off sets its own 120 cm target) and keep the
throttle centred: `increase_height()` / `decrease_height()` take over from the target.

**Needs firmware with MSP 227** (the ws28py build, and the MagisV2 CG tilt branch once
DronaAviation/MagisV2#49 is released). Other firmware refuses it and the library prints a
message; nothing else is affected. The drone takes it only while flying in altitude hold:
not before take-off, while landing (a landing can't be undone from Python) or during a
flip. A target above the `set_max_height()` ceiling is lowered under it. To go down, use
`land()`.

`is_height_accepted()` tells whether the drone took the last target: `True`, `False` if it
refused it, `None` until it has answered (about 50 ms).

```python
pluto.take_off()
time.sleep(4)
pluto.set_height(80)    # hold 80 cm
time.sleep(0.2)
print(pluto.is_height_accepted())   # True: the drone is on its way to 80 cm
```

#### `RGB LED strip` (WS2812B)

Controls a WS2812B LED strip on the drone. **Needs firmware with MSP 225/226** (the ws28py
build today); other firmware refuses it with a printed message. Call `led_init()` before
arming.

```python
pluto.led_init(8)                   # take the strip: 8 LEDs (1-64)
pluto.set_led(0, 255, 0, 0)         # LED 0 red (r, g, b 0-255)
pluto.set_led(1, 0, 0, 255, 50)     # LED 1 blue at 50 % brightness
pluto.set_all_leds(0, 255, 0)       # every LED green
pluto.wait_leds()                   # wait until everything queued has been sent
pluto.led_release()                 # turn it off and give it back to the drone
```

LED packets take the place of a telemetry request in the library's update cycle, so they
never add to the drone's command load. Calls return at once.

#### `Get MSP_Analog Values`

```python
#Returns the battery value in volts from the drone (None if no data).
battery = pluto.get_battery()

#Returns the battery percentage from the drone (None if no data).
battery_percentage = pluto.get_battery_percentage()
```

## Enhanced Battery Telemetry (Mobile App Features)

The package supports enhanced battery telemetry matching the mobile app, on MagisV2 and on
older Magis firmware.

**MagisV2 sends its battery values ready to use.** State of charge, mAh drawn and
remaining, and the battery warning come from the drone's own battery monitor and are
returned exactly as sent; only the voltage is converted from mV to V. The battery capacity
the drone uses for them is stored in the drone, not in the library.

Every battery method returns `None` when there is no current data (not connected, or
nothing from the drone for about a second), and for values the connected firmware does not
send. Nothing from an earlier connection is ever returned as current.

### Battery Capacity (MagisV2)

The drone stores its battery capacity and computes the state of charge and mAh from it
(600 mAh by default). Change it when you fit a different pack, e.g. 1200 mAh on a Pluto
Guru:

```python
capacity = pluto.get_battery_capacity()   # mAh, None if no reply
pluto.set_battery_capacity(1200)          # 100-5000 mAh, drone disarmed
```

**The drone uses a new capacity only after a restart, so saving it stops the flight
controller: its LEDs blink green and blue until you switch it off and on.**
`set_battery_capacity()` checks that the drone stopped, disconnects, and tells you to
power-cycle it. It writes nothing if the capacity is already set, and keeps the battery
voltages stored with it. Do it on the ground, never in flight (it is refused while armed).

### Protocol Version Detection

The firmware is detected automatically from the battery reply (10 bytes on MagisV2, 7 on
older Magis):

```python
# 1 = MagisV2 battery layout, 0 = older Magis
print(f"Protocol Version: {pluto.MSP_Protocol_Version}")
```

### Comprehensive Battery Information

Get all battery information in one call:

```python
#Get comprehensive battery info (all parameters), None if no data
battery_info = pluto.get_battery_info()

# MagisV2 returns:
# {
#     'voltage': '4.10V',
#     'current': '350mA',
#     'capacity_drawn': '51mAh',
#     'capacity_remaining': '549mAh',
#     'state_of_charge': '91%',
#     'auto_land_mode': 0,
#     'battery_warning': 'OK',
#     'protocol': 'V1 (Enhanced)'
# }
```

### Battery Helper Methods
Convenient methods to check battery status without parsing raw values.

```python
# Get status string ('FULL', 'TWO_BAR', 'ONE_BAR', 'EMPTY'), None if no data
status = pluto.get_battery_level_status()

# The drone's own battery warning: 'OK', 'WARNING' or 'CRITICAL'
warning = pluto.get_battery_warning()

# Check for critical battery (< 20%)
if pluto.is_battery_critical():
    print("LAND NOW!")

# True when the drone warns about its battery (WARNING or CRITICAL): land now
if pluto.should_auto_land():
    print("Battery low: land now")

# Get a full summary dict (for UI/Display), built from one reading; None if no data
summary = pluto.get_battery_status_summary()
if summary is not None:
    print(f"Icon: {summary['icon']} | Voltage: {summary['voltage']}")
```

The CG tilt firmware reports CRITICAL as WARNING while the drone is armed: it is then
landing by itself and the app only warns. No reply tells "auto-landing" apart, so
`should_auto_land()` means "the drone warns about its battery". `get_auto_land_status_text()`
still works and now returns the same as `get_battery_warning()` (it used to say `NORMAL`,
`LOW_BATTERY_WARNING` or `AUTO_LANDING`).

### Individual Telemetry Parameters

#### MagisV2 (Primus X2 / V5 boards)

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

#Get the battery warning code: 0 OK, 1 WARNING, 2 CRITICAL
code = pluto.get_auto_land_status()
```

#### Older Magis (Pluto / V4 boards)

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

**MagisV2 (`MSP_Protocol_Version` 1):**
- `vBatComp`: Battery voltage, compensated (sent in mV, returned in volts)
- `mAmpRaw`: Current draw in milliamps
- `mAhDrawn`: Capacity consumed in milliamp-hours
- `mAhRemain`: Remaining capacity in milliamp-hours
- `soc_Fused`: State of charge percentage (0-100)
- `auto_LandMode`: Battery warning code (0 OK, 1 WARNING, 2 CRITICAL)

**Older Magis (`MSP_Protocol_Version` 0):**
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

The `keyboard` package needs administrator rights on Linux (run with `sudo`) and the
Accessibility permission for your terminal on macOS. On Windows it works as is.

```python
from plutocontrol import Pluto
import time
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
        time.sleep(0.05)  # check the keys 20 times a second, not millions

control_loop()
pluto.disarm()
pluto.disconnect()
```

