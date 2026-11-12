#!/usr/bin/env python3
"""
Fly a Pluto drone from the keyboard and watch its telemetry live.

    pip install pygame-ce          # "import pygame"; plain pygame has no build for Python 3.14
    python examples/keyboard_control.py               # drone at 192.168.4.1
    python examples/keyboard_control.py 192.168.4.1   # IP given (port 23)

A small window opens: click it so it has the keyboard focus.

Hold to move. Each stick is 1500 while its key is up and moves only while you hold it:
    W / S          forward / backward
    A / D          left / right
    Left / Right   turn (yaw)
Press once:
    Space          take off                   L        land
    X              DISARM: motors stop at once
    }  /  {        height target +20 / -20 cm ( ] and [ work too )
    1 / 2 / 3      LEDs red / green / blue    0        LEDs off
    Q              quit                       Ctrl+C in the terminal: disarm and quit

- Keys count only while the window has the focus: typing in another app never moves the
  drone, and if the window loses the focus every stick goes back to 1500.
- The height keys need firmware with MSP 227, the LED keys firmware with MSP 225/226.
  Other firmware refuses them and the reason shows under "Messages". Next to the height
  target the screen shows whether the drone accepted or refused it: it takes a target
  only while flying in altitude hold, not before take-off or while landing.
- The script takes the LED strip at start (the drone only hands it over while disarmed)
  and gives it back when you quit.

Try it with the propellers off first.
"""
import os
import sys
import threading
import time
from collections import deque

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
os.environ.setdefault("SDL_NO_SIGNAL_HANDLERS", "1")  # Ctrl+C stays a KeyboardInterrupt
try:
    import pygame
except ImportError:
    sys.exit("This example needs pygame:  pip install pygame-ce")

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.isfile(os.path.join(_REPO, "plutocontrol", "Pluto.py")):
    sys.path.insert(0, _REPO)  # in a source checkout: test this code, not an installed copy

from plutocontrol import Pluto  # noqa: E402

HEIGHT_STEP_CM = 20      # height change per { or } press
HEIGHT_LIMIT_CM = 300    # highest target } sets: raise it if you have the room
HEIGHT_MIN_CM = 20       # lowest target plutocontrol accepts (land() goes lower)
TAKEOFF_HEIGHT_CM = 120  # MagisV2 take-off climbs to this height and holds it
LED_COUNT = 16           # LEDs on your strip
LED_BRIGHTNESS = 50      # %
ARM_AUX_MIN = 1300       # AUX4 at or above this is the arm switch "on"
RC_CENTER = 1500

# Stick, key that moves it up, key that moves it down, and the plutocontrol call for each
AXES = [("rcPitch", pygame.K_w, pygame.K_s, "forward", "backward"),
        ("rcRoll", pygame.K_d, pygame.K_a, "right", "left"),
        ("rcYaw", pygame.K_RIGHT, pygame.K_LEFT, "right_yaw", "left_yaw")]
COLOURS = {"1": ("red", (255, 0, 0)), "2": ("green", (0, 255, 0)),
           "3": ("blue", (0, 0, 255)), "0": ("off", (0, 0, 0))}
# Shown after the height target: pluto.is_height_accepted() for a target sent with { / }
NOT_SENT = "not sent"
HEIGHT_NOTES = {True: "  accepted", False: "  REFUSED by drone", None: "  waiting"}

HELP = [
    "Hold:  W/S forward/back   A/D left/right   Left/Right arrows turn",
    "Press: Space take off   L land   X DISARM (motors stop)",
    "       }} / {{  height +{0} / -{0} cm   1 red  2 green  3 blue  0 LEDs off".format(HEIGHT_STEP_CM),
    "       Q quit   (Ctrl+C in the terminal: disarm and quit)",
]

BACKGROUND = (22, 24, 28)
TEXT = (230, 232, 236)
DIM = (140, 146, 156)
ALERT = (255, 110, 100)
LINE_HEIGHT = 22


class MessageLog:
    """Stands in for sys.stdout: keeps the last lines the library and this script print."""

    def __init__(self, keep=6):
        self.lines = deque(maxlen=keep)  # [text, repeat count]
        self.muted_thread = None         # prints from this thread are dropped
        self._partial = ""
        self._lock = threading.Lock()

    def write(self, text):
        if threading.current_thread() is self.muted_thread:
            return len(text)
        with self._lock:
            self._partial += text
            while "\n" in self._partial:
                line, self._partial = self._partial.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                if self.lines and self.lines[-1][0] == line:
                    self.lines[-1][1] += 1
                else:
                    self.lines.append([line, 1])
        return len(text)

    def flush(self):
        pass

    def recent(self):
        with self._lock:
            return [text if n == 1 else "{}  (x{})".format(text, n) for text, n in self.lines]


class Telemetry(threading.Thread):
    """Reads the drone's telemetry about 5 times a second, away from the keyboard loop."""

    def __init__(self, pluto):
        super().__init__(daemon=True)
        self.pluto = pluto
        self.values = {}
        self.running = True

    def run(self):
        p = self.pluto
        readers = [("status", p.get_flight_status), ("volts", p.get_battery),
                   ("soc", p.get_state_of_charge), ("warning", p.get_battery_warning),
                   ("height", p.get_height), ("vario", p.get_vario),
                   ("roll", p.get_roll), ("pitch", p.get_pitch), ("yaw", p.get_yaw)]
        while self.running:
            for key, read in readers:
                try:
                    self.values[key] = read()  # None without current data
                except Exception:
                    self.values[key] = None
            time.sleep(0.2)


def update_sticks(pluto, applied):
    """Each stick is 1500 unless its key is held right now. Sends only what changed."""
    held = pygame.key.get_pressed()
    for stick, up_key, down_key, up_move, down_move in AXES:
        direction = held[up_key] - held[down_key]  # both keys held: 1500
        if applied.get(stick) == direction:
            continue
        applied[stick] = direction
        if direction > 0:
            getattr(pluto, up_move)()
        elif direction < 0:
            getattr(pluto, down_move)()
        else:
            setattr(pluto, stick, RC_CENTER)


def show(value, spec="{}"):
    return "--" if value is None else spec.format(value)


def draw(window, font, pluto, ip, t, target, height_result, led, log):
    link_ok = pluto.connected
    warning = t.get("warning")
    rows = [
        ("Pluto keyboard control   {}   link: {}".format(
            ip, "connected" if link_ok else "LOST: press Q, then run again"), TEXT if link_ok else ALERT),
        ("", TEXT),
        ("Status    {}".format(show(t.get("status"))), TEXT),
        ("Battery   {} V   {} %   {}".format(
            show(t.get("volts"), "{:.2f}"), show(t.get("soc")), show(warning)),
         ALERT if warning in ("WARNING", "CRITICAL") else TEXT),
        ("Height    {} cm   target {}{}   climb {} cm/s".format(
            show(t.get("height")), show(target, "{} cm"), HEIGHT_NOTES.get(height_result, ""),
            show(t.get("vario"))),
         ALERT if height_result is False else TEXT),
        ("Attitude  roll {} deg   pitch {} deg   yaw {} deg".format(
            show(t.get("roll"), "{:.1f}"), show(t.get("pitch"), "{:.1f}"),
            show(t.get("yaw"), "{:.0f}")), TEXT),
        ("Sticks    roll {}  pitch {}  yaw {}  throttle {}  arm switch {}".format(
            pluto.rcRoll, pluto.rcPitch, pluto.rcYaw, pluto.rcThrottle,
            "on" if pluto.rcAUX4 >= ARM_AUX_MIN else "off"), TEXT),
        ("LEDs      {}".format(led), TEXT),
        ("", TEXT),
    ] + [(line, DIM) for line in HELP] + [("", TEXT), ("Messages", DIM)] + \
        [("  " + m, TEXT) for m in log.recent()]

    window.fill(BACKGROUND)
    for i, (text, colour) in enumerate(rows):
        if text:
            window.blit(font.render(text, True, colour), (16, 14 + i * LINE_HEIGHT))
    pygame.display.flip()


def run(pluto, ip, window, font, log, telemetry):
    target = None           # height target shown, cm
    accepted_target = None  # last target the drone took ( or the take-off height ): { / } step from it
    height_sent = False     # target was sent with { / }: show whether the drone took it
    led = "not set"
    applied = {}     # stick -> direction last sent (1, 0, -1)
    quit_asked = 0.0
    clock = pygame.time.Clock()
    frame = 0

    def maybe_armed():
        # Only these states say for sure whether the drone is armed. Crash, signal loss and
        # the low-battery states don't ( a critical battery is reported while armed on some
        # firmware and after landing on others ), so then go by the arm switch we send.
        status = telemetry.values.get("status")
        if status == "ARMED":
            return True
        if status in ("OK_TO_ARM", "NOT_OK_TO_ARM", "ACCEL_GYRO_CALIBRATION", "MAG_CALIBRATION"):
            return False
        return pluto.rcAUX4 >= ARM_AUX_MIN

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
                now = time.monotonic()
                if not pluto.connected or not maybe_armed() or now - quit_asked < 3.0:
                    return
                quit_asked = now
                print("The drone is armed: L lands, X disarms. Press Q again to quit anyway.")
            elif event.type == pygame.KEYDOWN:
                key, char = event.key, event.unicode
                if key == pygame.K_SPACE:
                    pluto.take_off()
                    target = accepted_target = TAKEOFF_HEIGHT_CM
                    height_sent = False
                    applied.clear()  # take_off() centred the sticks: send held keys again
                elif key == pygame.K_l:
                    pluto.land()
                    target = accepted_target = None
                    height_sent = False
                elif key == pygame.K_x:
                    pluto.disarm()
                    target = accepted_target = None
                    height_sent = False
                elif key in (pygame.K_RIGHTBRACKET, pygame.K_LEFTBRACKET) or char in ("}", "{"):
                    up = key == pygame.K_RIGHTBRACKET or char == "}"
                    base = accepted_target
                    if base is None:
                        base = telemetry.values.get("height") or TAKEOFF_HEIGHT_CM
                    step = HEIGHT_STEP_CM if up else -HEIGHT_STEP_CM
                    target = max(HEIGHT_MIN_CM, min(HEIGHT_LIMIT_CM, base + step))
                    pluto.set_height(target)
                    height_sent = True
                    print("Height target {} cm".format(target))
                elif char in COLOURS:
                    led, rgb = COLOURS[char]
                    pluto.set_all_leds(*rgb, brightness=LED_BRIGHTNESS)
                    print("LEDs {}".format(led))

        update_sticks(pluto, applied)
        height_result = pluto.is_height_accepted() if height_sent else NOT_SENT
        if height_result is True:
            accepted_target = target
        if frame % 5 == 0:  # screen at 10 Hz, keys at 50 Hz
            draw(window, font, pluto, ip, telemetry.values, target, height_result, led, log)
        frame += 1
        clock.tick(50)


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.4.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 23

    pluto = Pluto(ip, port)
    if not pluto.connect():
        sys.exit(1)

    pygame.init()
    window = pygame.display.set_mode((760, 30 + LINE_HEIGHT * 22))
    pygame.display.set_caption("Pluto keyboard control")
    font = pygame.font.SysFont("menlo,consolas,dejavusansmono,couriernew,monospace", 16)

    terminal, log = sys.stdout, MessageLog()
    sys.stdout = log  # library prints go to the Messages panel
    telemetry = Telemetry(pluto)
    log.muted_thread = telemetry  # the getters print every value they read: drop those
    telemetry.start()
    pluto.led_init(LED_COUNT)

    emergency = False
    try:
        run(pluto, ip, window, font, log, telemetry)
    except KeyboardInterrupt:
        emergency = True
        pluto.disarm()
    finally:
        sys.stdout = terminal
        telemetry.running = False
        if emergency:
            time.sleep(0.1)  # let the disarm frame go out before the link closes
        pluto.led_release()
        pluto.disconnect()
        pygame.quit()
        for line in log.recent():
            print("  " + line)
        if emergency:
            print("Ctrl+C: disarmed (motors stopped) and disconnected.")


if __name__ == "__main__":
    main()
