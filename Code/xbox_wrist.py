#!/usr/bin/env python3
"""
Drive the differential wrist with an Xbox controller.

Setup (once):
    pip3 install pygame pyserial
    Pair the controller to the Mac (Bluetooth) or plug it in with USB.

Run (close the Arduino Serial Monitor first, only one program can own the port):
    python3 xbox_wrist.py              # finds /dev/cu.usbmodem* by itself
    python3 xbox_wrist.py /dev/cu.usbmodem1101
    python3 xbox_wrist.py --show       # print raw stick/button numbers (to fix the mapping)

Controls:
    Left stick up/down     pitch   (further = faster)
    Left stick left/right  roll
    RB / LB                max speed up / down
    A    go home (pitch 0, roll 0)
    B    stop
    Y    set zero here
    X    print status
    Start/Menu  enable / disable drivers
    Ctrl-C to quit (the wrist stops on its own within 0.3 s)
"""

import glob
import os
import sys
import threading
import time

# By default SDL (the library under pygame) only delivers controller input to the
# app that has keyboard focus. When this runs from Terminal, *Terminal* has focus,
# not Python, so on macOS the sticks read as 0 forever. This hint says "deliver it
# anyway". It must be set before pygame is imported/initialised.
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")

import pygame
import serial

# ---------------- mapping: run with --show if a stick/button does the wrong thing ----------------
AXIS_ROLL  = 0      # left stick X
AXIS_PITCH = 1      # left stick Y (pygame reports UP as negative, flipped below)
BTN_A, BTN_B, BTN_X, BTN_Y = 0, 1, 2, 3
BTN_LB, BTN_RB = 9, 10
BTN_START = 6
# Common alternative on some macOS/driver combos: LB=4, RB=5, START=7.

INVERT_PITCH = True     # stick up = pitch +
INVERT_ROLL  = False
DEADZONE   = 0.12       # ignore stick drift near center
EXPO       = 2.0        # 1 = linear, 2 = gentle near center (fine control)
SEND_HZ    = 50
# -----------------------------------------------------------------------------------------------

max_rate = 400           # deg/s at full stick; RB/LB step through RATE_STEPS
RATE_STEPS = [10, 20, 30, 45, 60, 90, 120,160,180,200,220,280,300,400]


def find_port():
    if len(sys.argv) > 1 and not sys.argv[1].startswith("--"):
        return sys.argv[1]
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("No /dev/cu.usbmodem* found. Is the Arduino plugged in?")
    return ports[0]


def shape(v):
    """Deadzone + expo so small stick moves give slow, precise motion."""
    if abs(v) < DEADZONE:
        return 0.0
    v = (abs(v) - DEADZONE) / (1 - DEADZONE) * (1 if v > 0 else -1)
    return (abs(v) ** EXPO) * (1 if v > 0 else -1)


def reader(ser):
    """Print whatever the Arduino says. Also keeps its USB buffer drained,
    which matters: if nobody reads, the Arduino's prints can stall its loop."""
    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print(f"\r[wrist] {line}")
        except serial.SerialException:
            return


def show_mode(js):
    print("Move sticks / press buttons. Ctrl-C to quit.")
    while True:
        pygame.event.pump()
        axes = " ".join(f"{i}:{js.get_axis(i):+.2f}" for i in range(js.get_numaxes()))
        btns = " ".join(str(i) for i in range(js.get_numbuttons()) if js.get_button(i))
        print(f"\raxes {axes}   buttons down: [{btns}]      ", end="", flush=True)
        time.sleep(0.05)


def main():
    global max_rate
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit("No controller found. Pair it / plug it in, then run again.")
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller: {js.get_name()}")

    if "--show" in sys.argv:
        show_mode(js)
        return

    port = find_port()
    ser = serial.Serial(port, 115200, timeout=0.1)
    print(f"Arduino: {port}")
    time.sleep(2)  # give the board a moment after the port opens
    threading.Thread(target=reader, args=(ser,), daemon=True).start()

    def send(cmd):
        ser.write((cmd + "\n").encode())

    print(__doc__.split("Controls:")[1])
    print(f"Max speed {max_rate:.0f} deg/s")

    stick_active_until = 0.0
    prev_buttons = {}
    enabled = False   # the wrist turns its drivers on by itself when you move
    period = 1.0 / SEND_HZ

    try:
        while True:
            t0 = time.time()
            pygame.event.pump()

            # ---- buttons: act once per press, not every frame ----
            def pressed(b):
                now = b < js.get_numbuttons() and js.get_button(b)
                was = prev_buttons.get(b, False)
                prev_buttons[b] = now
                return now and not was

            if pressed(BTN_A):     send("P0 R0"); stick_active_until = 0
            if pressed(BTN_B):     send("X")
            if pressed(BTN_Y):     send("Z")
            if pressed(BTN_X):     send("?")
            if pressed(BTN_START):
                enabled = not enabled
                send("E" if enabled else "D")
            faster, slower = pressed(BTN_RB), pressed(BTN_LB)
            if faster or slower:
                i = RATE_STEPS.index(max_rate)
                i = min(i + 1, len(RATE_STEPS) - 1) if faster else max(i - 1, 0)
                max_rate = RATE_STEPS[i]
                print(f"\rMax speed {max_rate} deg/s")

            # ---- sticks -> jog rates ----
            p = shape(js.get_axis(AXIS_PITCH)) * (-1 if INVERT_PITCH else 1)
            r = shape(js.get_axis(AXIS_ROLL))  * (-1 if INVERT_ROLL else 1)

            # Only send J while the stick is in use (plus a short tail of zeros
            # after release). Then the stream stops, so an A-button "go home"
            # isn't immediately overridden by J 0 0.
            if p or r:
                stick_active_until = t0 + 0.2
            if t0 < stick_active_until:
                send(f"J{p * max_rate:.1f} {r * max_rate:.1f}")

            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        send("X")
        print("\nbye")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
