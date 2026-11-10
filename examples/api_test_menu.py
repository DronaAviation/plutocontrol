#!/usr/bin/env python3
"""
plutocontrol API test menu: run any plutocontrol function by typing its number.

    python examples/api_test_menu.py               # asks for the drone's IP
    python examples/api_test_menu.py 192.168.4.1   # IP given (port 23)
    python examples/api_test_menu.py <ip> <port>   # IP and port given

- Type a number and press Enter to run that function. It asks for any values it needs;
  press Enter to take the default shown in [brackets].
- 'm' shows the menu again, 't' runs every read-only function once (quick self-test),
  'x' disarms at once, 'q' quits.
- Ctrl+C at any time disarms at once (motors stop) and returns to the menu.

Safety: try it on the ground with the propellers off first. Movement and height
commands make the drone fly.
"""
import io
import os
import sys
import time
from contextlib import redirect_stdout

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.isfile(os.path.join(_REPO, "plutocontrol", "Pluto.py")):
    sys.path.insert(0, _REPO)  # in a source checkout: test this code, not an installed copy

import plutocontrol  # noqa: E402
from plutocontrol import Pluto  # noqa: E402


# ── Input helpers ─────────────────────────────────────────────────────────────

def ask(prompt, kind=str, default=None, allow_none=False):
    """Ask for one value. Enter takes the default (or None when allow_none)."""
    if default is not None:
        hint = " [{}]".format(default)
    elif allow_none:
        hint = " [Enter = default]"
    else:
        hint = ""
    while True:
        text = input("  {}{}: ".format(prompt, hint)).strip()
        if text == "":
            if default is not None:
                return default
            if allow_none:
                return None
            continue
        try:
            return kind(text)
        except ValueError:
            print("  Not a valid {}, try again.".format("number" if kind is not str else "value"))


def confirm(text):
    return input("  {} [y/N]: ".format(text)).strip().lower() in ("y", "yes")


def quiet(fn, *args):
    """Call fn without the library's own prints (used by the monitor and the self-test)."""
    with redirect_stdout(io.StringIO()):
        return fn(*args)


# ── Actions that need more than one call ──────────────────────────────────────

def hold_then_reset(pluto, seconds):
    if seconds > 0:
        time.sleep(seconds)
        pluto.reset()
        print("  (held {:.1f} s, sticks centred again)".format(seconds))


def move(name):
    def run(p):
        strength = ask("strength 0-500", int, allow_none=True)
        seconds = ask("hold for seconds (0 = keep until reset)", float, 1.0)
        getattr(p, name)(strength)
        hold_then_reset(p, seconds)
    return run


def throttle(name):
    def run(p):
        seconds = ask("hold for seconds (0 = keep until reset)", float, 1.0)
        getattr(p, name)()
        hold_then_reset(p, seconds)
    return run


def take_off(p):
    if confirm("Clear space around the drone. Take off now?"):
        p.take_off()
        print("  On MagisV2 the drone climbs to about 120 cm and holds it.")


def motor_test(p):
    print("  Motor test: spins one motor while the drone is DISARMED.")
    if not confirm("Propellers OFF and hands clear?"):
        return
    index = ask("motor index 0-3", int, 0)
    speed = ask("speed 1000-2000 (1000 = stop)", int, 1100)
    p.motor_speed(index, speed)
    if speed != 1000:
        seconds = ask("stop after seconds (0 = keep spinning)", float, 2.0)
        if seconds > 0:
            time.sleep(seconds)
            p.motor_speed(index, 1000)
            print("  (motor stopped)")


def set_battery_capacity(p):
    print("  The drone uses a new capacity only after a restart: after saving it stops")
    print("  and blinks its LEDs until you switch it off and on. Drone must be disarmed.")
    capacity = ask("capacity in mAh (100-5000)", int, 600)
    if confirm("Save {} mAh to the drone now?".format(capacity)):
        return p.set_battery_capacity(capacity)
    return None


def calibrate(name):
    def run(p):
        print("  Disarm first. Place the drone on a flat, level surface and keep it still.")
        if name == "calibrate_magnetometer":
            print("  Then rotate the drone as for the app's compass calibration.")
        if confirm("Start?"):
            getattr(p, name)()
    return run


def trim(p):
    roll = ask("roll trim -1000..1000", int, 0)
    pitch = ask("pitch trim -1000..1000", int, 0)
    p.trim(roll, pitch)


def save_eeprom(p):
    print("  Saves the current trim so it survives a reboot (refused while armed).")
    if confirm("Save to EEPROM now?"):
        p.send_request_msp_eeprom_write()


def set_height(p):
    print("  Needs firmware with MSP 227. Works only while flying in altitude hold.")
    p.set_height(ask("height in cm (20-10000)", int, 100))


def set_max_height(p):
    value = ask("ceiling in cm (0-10000), Enter = remove the ceiling", int, allow_none=True)
    p.set_max_height(value)


def led_init(p):
    p.led_init(ask("number of LEDs (1-64)", int, 8))


def set_led(p):
    index = ask("LED index", int, 0)
    r = ask("red 0-255", int, 255)
    g = ask("green 0-255", int, 0)
    b = ask("blue 0-255", int, 0)
    brightness = ask("brightness 0-100 %", int, 100)
    p.set_led(index, r, g, b, brightness)


def set_all_leds(p):
    r = ask("red 0-255", int, 0)
    g = ask("green 0-255", int, 255)
    b = ask("blue 0-255", int, 0)
    brightness = ask("brightness 0-100 %", int, 100)
    p.set_all_leds(r, g, b, brightness)


def wait_leds(p):
    return p.wait_leds(ask("timeout in seconds", float, 3.0))


def monitor(p):
    seconds = ask("show live telemetry for seconds", float, 10.0)
    print("  height cm | roll  pitch   yaw | vario | battery V  SoC %  warning | status")
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        line = "  {:>9} | {:>5} {:>6} {:>5} | {:>5} | {:>9} {:>6}  {:>7} | {}".format(
            *[str(quiet(f)) for f in (p.get_height, p.get_roll, p.get_pitch, p.get_yaw, p.get_vario,
                                      p.get_battery, p.get_state_of_charge, p.get_battery_warning,
                                      p.get_flight_status)])
        print(line)
        time.sleep(0.5)


# ── The menu ──────────────────────────────────────────────────────────────────
# (group, label, action). An action is a method name (called with no arguments)
# or a function taking the Pluto object. Its return value is printed.

MENU = [
    ("Connection", "connect()", "connect"),
    ("Connection", "disconnect()", "disconnect"),
    ("Arming", "arm()", "arm"),
    ("Arming", "disarm()", "disarm"),
    ("Arming", "box_arm()", "box_arm"),
    ("Arming", "take_off()", take_off),
    ("Arming", "land()", "land"),
    ("Movement", "forward(strength)", move("forward")),
    ("Movement", "backward(strength)", move("backward")),
    ("Movement", "left(strength)", move("left")),
    ("Movement", "right(strength)", move("right")),
    ("Movement", "left_yaw(strength)", move("left_yaw")),
    ("Movement", "right_yaw(strength)", move("right_yaw")),
    ("Movement", "increase_height()", throttle("increase_height")),
    ("Movement", "decrease_height()", throttle("decrease_height")),
    ("Movement", "reset()", "reset"),
    ("Movement", "rc_values()", "rc_values"),
    ("Height", "set_height(cm)", set_height),
    ("Height", "set_max_height(cm)", set_max_height),
    ("Height", "get_max_height()", "get_max_height"),
    ("Telemetry", "get_height()", "get_height"),
    ("Telemetry", "get_vario()", "get_vario"),
    ("Telemetry", "get_roll()", "get_roll"),
    ("Telemetry", "get_pitch()", "get_pitch"),
    ("Telemetry", "get_yaw()", "get_yaw"),
    ("Telemetry", "get_acc_x()", "get_acc_x"),
    ("Telemetry", "get_acc_y()", "get_acc_y"),
    ("Telemetry", "get_acc_z()", "get_acc_z"),
    ("Telemetry", "get_gyro_x()", "get_gyro_x"),
    ("Telemetry", "get_gyro_y()", "get_gyro_y"),
    ("Telemetry", "get_gyro_z()", "get_gyro_z"),
    ("Telemetry", "get_mag_x()", "get_mag_x"),
    ("Telemetry", "get_mag_y()", "get_mag_y"),
    ("Telemetry", "get_mag_z()", "get_mag_z"),
    ("Telemetry", "live telemetry monitor", monitor),
    ("Status", "get_flight_status()", "get_flight_status"),
    ("Status", "is_armed()", "is_armed"),
    ("Battery", "get_battery()", "get_battery"),
    ("Battery", "get_battery_percentage()", "get_battery_percentage"),
    ("Battery", "get_analog_telemetry()", "get_analog_telemetry"),
    ("Battery", "get_battery_voltage_compensated()", "get_battery_voltage_compensated"),
    ("Battery", "get_current_mA()", "get_current_mA"),
    ("Battery", "get_capacity_drawn_mAh()", "get_capacity_drawn_mAh"),
    ("Battery", "get_capacity_remaining_mAh()", "get_capacity_remaining_mAh"),
    ("Battery", "get_state_of_charge()", "get_state_of_charge"),
    ("Battery", "get_battery_warning()", "get_battery_warning"),
    ("Battery", "get_auto_land_status()", "get_auto_land_status"),
    ("Battery", "get_auto_land_status_text()", "get_auto_land_status_text"),
    ("Battery", "get_battery_level_status()", "get_battery_level_status"),
    ("Battery", "is_battery_low()", "is_battery_low"),
    ("Battery", "is_battery_critical()", "is_battery_critical"),
    ("Battery", "should_auto_land()", "should_auto_land"),
    ("Battery", "get_battery_info()", "get_battery_info"),
    ("Battery", "get_battery_status_summary()", "get_battery_status_summary"),
    ("Battery", "get_rssi()  (older Magis)", "get_rssi"),
    ("Battery", "get_power_meter_sum()  (older Magis)", "get_power_meter_sum"),
    ("Battery", "get_battery_capacity()", "get_battery_capacity"),
    ("Battery", "set_battery_capacity(mAh)", set_battery_capacity),
    ("Calibration", "calibrate_acceleration()", calibrate("calibrate_acceleration")),
    ("Calibration", "calibrate_magnetometer()", calibrate("calibrate_magnetometer")),
    ("Calibration", "trim(roll, pitch)", trim),
    ("Calibration", "save trim to EEPROM", save_eeprom),
    ("Motor test", "motor_speed(index, speed)", motor_test),
    ("LEDs", "led_init(count)", led_init),
    ("LEDs", "set_led(index, r, g, b)", set_led),
    ("LEDs", "set_all_leds(r, g, b)", set_all_leds),
    ("LEDs", "wait_leds()", wait_leds),
    ("LEDs", "led_release()", "led_release"),
    ("Developer", "devOn()", "devOn"),
    ("Developer", "devOff()", "devOff"),
]

# Read-only functions run by the 't' self-test
SELF_TEST = [
    "rc_values", "get_height", "get_vario", "get_roll", "get_pitch", "get_yaw",
    "get_acc_x", "get_acc_y", "get_acc_z", "get_gyro_x", "get_gyro_y", "get_gyro_z",
    "get_mag_x", "get_mag_y", "get_mag_z", "get_flight_status", "is_armed", "get_max_height",
    "get_battery", "get_battery_percentage", "get_analog_telemetry", "get_battery_voltage_compensated",
    "get_current_mA", "get_capacity_drawn_mAh", "get_capacity_remaining_mAh", "get_state_of_charge",
    "get_battery_warning", "get_auto_land_status", "get_auto_land_status_text",
    "get_battery_level_status", "is_battery_low", "is_battery_critical", "should_auto_land",
    "get_battery_info", "get_battery_status_summary", "get_rssi", "get_power_meter_sum",
    "get_battery_capacity",
]


def show_menu():
    print()
    group = None
    for number, (grp, label, _) in enumerate(MENU, 1):
        if grp != group:
            group = grp
            print("\n  {}".format(grp))
        print("    {:>2}. {}".format(number, label))
    print("\n  t = self-test (every read-only function)   m = menu   x = DISARM NOW   q = quit")


def run_action(p, action):
    try:
        result = getattr(p, action)() if isinstance(action, str) else action(p)
    except AttributeError as e:
        print("  This plutocontrol version does not have it ({}). Update the package.".format(e))
        return
    if result is not None:
        print("  -> {!r}".format(result))
    elif isinstance(action, str) and action.startswith(("get_", "is_", "should_")):
        print("  -> None (no data: not connected, or nothing from the drone)")
    else:
        time.sleep(0.3)  # a refusal from the drone is printed by the background reader: show it here


def self_test(p):
    print("  {:<34} result".format("function"))
    missing = 0
    for name in SELF_TEST:
        if not hasattr(p, name):
            print("  {:<34} (not in this plutocontrol version)".format(name + "()"))
            continue
        value = quiet(getattr(p, name))
        missing += value is None
        print("  {:<34} {!r}".format(name + "()", value))
    print("  {} functions, {} returned None".format(len(SELF_TEST), missing))
    if missing:
        print("  (None is expected for values this firmware does not send, e.g. get_rssi on MagisV2)")


def emergency_disarm(p):
    print("\n  !! DISARM: motors stop.")
    p.disarm()
    time.sleep(0.2)  # let the background thread send it


def quit_tool(p):
    if p.connected and p.rcAUX4 >= 1300:
        if confirm("The drone may be armed. Land and disarm before quitting?"):
            p.land()
            print("  Landing...")
            time.sleep(5)
            p.disarm()
            time.sleep(0.3)
    if p.connected:
        p.disconnect()
    print("  Bye.")


def main():
    print(__doc__)
    print("Testing plutocontrol from {}\n".format(os.path.dirname(plutocontrol.__file__)))
    ip = sys.argv[1] if len(sys.argv) > 1 else ask("Drone IP", str, "192.168.4.1")
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 23
    pluto = Pluto(ip=ip, port=port)
    if confirm("Control through the camera module's Wi-Fi (cam())?"):
        pluto.cam()
    if not confirm("Connect now?") or not pluto.connect():
        print("  Not connected. Use option 1 to connect.")
    show_menu()

    while True:
        try:
            choice = input("\nEnter a number (m = menu): ").strip().lower()
            if choice == "q":
                quit_tool(pluto)
                return
            if choice == "m":
                show_menu()
            elif choice == "x":
                emergency_disarm(pluto)
            elif choice == "t":
                self_test(pluto)
            elif choice.isdigit() and 1 <= int(choice) <= len(MENU):
                _, label, action = MENU[int(choice) - 1]
                print("  {}".format(label))
                if not pluto.connected and label not in ("connect()", "disconnect()"):
                    print("  (not connected: option 1 connects)")
                run_action(pluto, action)
            elif choice:
                print("  Unknown choice. Enter 1-{}, t, m, x or q.".format(len(MENU)))
        except KeyboardInterrupt:
            emergency_disarm(pluto)
        except EOFError:
            quit_tool(pluto)
            return
        except Exception as e:  # keep the menu alive whatever a call does
            print("  Error: {!r}".format(e))


if __name__ == "__main__":
    main()
