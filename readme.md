Sure! Here is the `README.md` file formatted as requested:

```markdown
# PlutoLibrary

PlutoLibrary is a Python library for controlling Pluto drones. This library provides various methods to interact with the drone, including connecting, controlling movements, and accessing sensor data.

## Installation

To install the PlutoLibrary locally, navigate to the root directory of your package (where `setup.py` is located) and run:

```bash
pip install pypluto
```

## Usage

After installing the package, you can import and use the `Pluto` class in your Python scripts.

### Example Usage

#### Example 1

```python
from PlutoLibrary import Pluto

# Create an instance of the Pluto class
pluto = Pluto()

# Connect to the drone
pluto.connect()

# Arm the drone
pluto.arm()

# Perform other operations as needed
pluto.forward()
pluto.left()
pluto.increase_height()

# Disarm the drone
pluto.disarm()

# Disconnect from the drone
pluto.disconnect()
```

#### Example 2

```python
import time
from PlutoLibrary import Pluto

# Create an instance of the Pluto class
pluto = Pluto()

# Connect to the drone
pluto.connect()

# Take off the drone
pluto.take_off()

# Wait for 10 seconds while the drone hovers
time.sleep(10)

# Land the drone
pluto.land()

# Disconnect from the drone
pluto.disconnect()
```

## Class and Methods

### Pluto Class

#### `__init__()`
Initializes the Pluto object with default RC values and starts the write function in a separate thread.

#### `connect()`
Connects to the drone server.

```python
pluto.connect()
```

#### `disconnect()`
Disconnects from the drone server.

```python
pluto.disconnect()
```

#### `set_camera_ip_port(ip, port)`
Sets the IP and port for the camera connection.

```python
pluto.set_camera_ip_port('192.168.0.2', 9060)
```

#### `arm()`
Arms the drone, setting it to a ready state.

```python
pluto.arm()
```

#### `box_arm()`
Arms the drone with the throttle set to 1800.

```python
pluto.box_arm()
```

#### `disarm()`
Disarms the drone, stopping all motors.

```python
pluto.disarm()
```

#### `forward()`
Sets the drone to move forward.

```python
pluto.forward()
```

#### `backward()`
Sets the drone to move backward.

```python
pluto.backward()
```

#### `left()`
Sets the drone to move left (roll).

```python
pluto.left()
```

#### `right()`
Sets the drone to move right (roll).

```python
pluto.right()
```

#### `left_yaw()`
Sets the drone to yaw left.

```python
pluto.left_yaw()
```

#### `right_yaw()`
Sets the drone to yaw right.

```python
pluto.right_yaw()
```

#### `reset()`
Resets the RC values to the default state.

```python
pluto.reset()
```

#### `increase_height()`
Increases the drone's height.

```python
pluto.increase_height()
```

#### `decrease_height()`
Decreases the drone's height.

```python
pluto.decrease_height()
```

#### `take_off()`
Arms the drone and prepares it for takeoff.

```python
pluto.take_off()
```

#### `land()`
Commands the drone to land.

```python
pluto.land()
```

#### `rc_values()`
Returns the current RC values.

```python
values = pluto.rc_values()
```

#### `motor_speed(motor_index, speed)`
Sets the speed of a specific motor (motor index from 0 to 3).

```python
pluto.motor_speed(0, 1200)
```

#### `trim_left_roll()`
Trims the left roll by increasing the RC roll value.

```python
pluto.trim_left_roll()
```

#### `get_height()`
Returns the height of the drone from the sensors.

```python
height = pluto.get_height()
```

#### `get_vario()`
Returns the rate of change of altitude from the sensors.

```python
vario = pluto.get_vario()
```

#### `get_roll()`
Returns the roll value from the drone.

```python
roll = pluto.get_roll()
```

#### `get_pitch()`
Returns the pitch value from the drone.

```python
pitch = pluto.get_pitch()
```

#### `get_yaw()`
Returns the yaw value from the drone.

```python
yaw = pluto.get_yaw()
```

#### `get_acc_x()`
Returns the accelerometer value for the x-axis.

```python
acc_x = pluto.get_acc_x()
```

#### `get_acc_y()`
Returns the accelerometer value for the y-axis.

```python
acc_y = pluto.get_acc_y()
```

#### `get_acc_z()`
Returns the accelerometer value for the z-axis.

```python
acc_z = pluto.get_acc_z()
```

#### `get_gyro_x()`
Returns the gyrometer value for the x-axis.

```python
gyro_x = pluto.get_gyro_x()
```

#### `get_gyro_y()`
Returns the gyrometer value for the y-axis.

```python
gyro_y = pluto.get_gyro_y()
```

#### `get_gyro_z()`
Returns the gyrometer value for the z-axis.

```python
gyro_z = pluto.get_gyro_z()
```

#### `get_mag_x()`
Returns the magnetometer value for the x-axis.

```python
mag_x = pluto.get_mag_x()
```

#### `get_mag_y()`
Returns the magnetometer value for the y-axis.

```python
mag_y = pluto.get_mag_y()
```

#### `get_mag_z()`
Returns the magnetometer value for the z-axis.

```python
mag_z = pluto.get_mag_z()
```

#### `calibrate_acceleration()`
Calibrates the accelerometer.

```python
pluto.calibrate_acceleration()
```

#### `calibrate_magnetometer()`
Calibrates the magnetometer.

```python
pluto.calibrate_magnetometer()
```

#### `get_battery()`
Returns the battery value in volts from the drone.

```python
battery = pluto.get_battery()
```

#### `get_battery_percentage()`
Returns the battery percentage from the drone.

```python
battery_percentage = pluto.get_battery_percentage()
```

#### `send_packet(buff)`
Sends a packet to the drone.

```python
pluto.send_packet(buff)
```

#### `receive_packet()`
Receives a packet from the drone.

```python
packet = pluto.receive_packet()
```

#### `read16(arr)`
Unpacks a byte array to extract 16-bit values.

```python
value = pluto.read16(arr)
```

With this `README.md`, users will have a comprehensive understanding of the library's functionality and how to use it effectively.
```