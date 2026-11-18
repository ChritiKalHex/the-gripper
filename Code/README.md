# Differential Wrist — Xbox controlled

A 2-axis (pitch + roll) 3D-printed bevel-gear differential wrist. Two NEMA 23 steppers on
Lin Engineering R208 drivers are driven by an Arduino UNO R4, and an Xbox controller
connected to a Mac steers it through a small Python script over USB serial.

```
Xbox controller --Bluetooth--> Mac (xbox_wrist.py) --USB serial--> Arduino UNO R4
                                                                     |  STEP/DIR
                                                          R208 A  <--+-->  R208 B
                                                            |               |
                                                         NEMA 23 A      NEMA 23 B
                                                             \             /
                                                           differential wrist
```

**Mechanical design:** 3DPDiffWrist by **tiago** (Printables),
CC BY-SA 4.0, DOI [10.5281/zenodo.23287814](https://doi.org/10.5281/zenodo.23287814).
Any published remix of this project must credit tiago and stay CC BY-SA.

## Files

| File | What it does |
|---|---|
| `diff_wrist_xbox.ino` | Arduino firmware: serial commands (`P`/`R` moves) plus `J` jog mode for the controller |
| `~/Downloads/xbox_wrist.py` | Mac side: reads the controller, sends jog commands to the Arduino |
| `~/Downloads/basic_stepper_test/` | Test sketch, motor A only (see Troubleshooting) |

## Hardware

- Arduino UNO R4 (Minima or WiFi)
- 2× NEMA 23 stepper, 1.8°/step
- 2× Lin Engineering R208 driver (DB-9), 24 V supply
- Xbox Wireless Controller paired to the Mac over Bluetooth

### Wiring

| Signal | Arduino pin |
|---|---|
| Motor A STEP / DIR | 2 / 3 |
| Motor B STEP / DIR | 4 / 5 |
| Disable | not wired (drivers always on) |

The R208 inputs are opto-isolated and **sinking**: connect the R208's **Opto Supply** to the
Arduino's **5V**. An input turns on when the Arduino pin goes LOW. Do **not** join the
Arduino GND to the R208 grounds; that would defeat the isolation.

Set the R208 current pot **low** at first. The R208 tops out around 2 A, and the printed
PLA gears strip easily.

### Gearing

Motor pinion 11T → side bevel 53T (4.818 : 1), side bevel → output bevel 53T (1 : 1).

```
steps per joint degree = 200 × MICROSTEPS × (53 / 11) / 360
```

`MICROSTEPS` in the firmware **must match the R208's SR1/SR2 jumpers**, or every move comes
out the wrong size.

## Setup

### 1. Arduino

1. Install the **AccelStepper** library (Arduino IDE → Library Manager).
2. Open `diff_wrist_xbox.ino`, check the CONFIG section (pins, `MICROSTEPS`, joint limits).
3. Upload to the UNO R4.

### 2. Mac

```bash
pip3 install pygame pyserial
```

### 3. Controller

1. Pair the Xbox controller in **System Settings → Bluetooth** (hold the pairing button on
   top of the controller until the logo flashes fast).
2. Before each session, press the Xbox button until the logo is **solid**.

Use **Bluetooth, not a USB cable.** Over USB, pygame detects the controller but every
stick reads 0.00 and no buttons register.

## Running it

1. Put the wrist at its neutral pose **by hand**. There are no endstops.
2. Close the Arduino Serial Monitor (only one program can use the port).
3. Run:

```bash
python3 ~/Downloads/xbox_wrist.py
```

It finds the Arduino (`/dev/cu.usbmodem*`) by itself. To name the port yourself:
`python3 ~/Downloads/xbox_wrist.py /dev/cu.usbmodemXXXX`.

4. Press **Y** to set the current pose as zero, then move the left stick.

### Controls

| Input | Action |
|---|---|
| Left stick up/down | pitch (further = faster) |
| Left stick left/right | roll |
| RB / LB | max speed up / down (steps through `RATE_STEPS`) |
| A | go home (pitch 0, roll 0) |
| B | stop |
| Y | set zero here |
| X | print status |
| Start / Menu | enable / disable drivers |
| Ctrl-C | quit (the wrist stops on its own within 0.3 s) |

The stick has a deadzone (0.12) and an expo curve, so small movements give slow, fine
control. If no jog command arrives for 300 ms (script closed, cable pulled), the firmware
ramps the wrist to a stop.

### Testing the controller only

```bash
python3 ~/Downloads/xbox_wrist.py --show
```

Prints raw axis values and pressed button numbers without touching the Arduino. Use it to
check the controller is talking, or to fix the button/axis numbers at the top of
`xbox_wrist.py`.

## Serial commands

You can also type these into the Arduino Serial Monitor (115200 baud, newline):

| Command | Meaning |
|---|---|
| `P<deg>` | move pitch to an angle, e.g. `P30` |
| `R<deg>` | move roll to an angle, e.g. `R-45` |
| `P<deg> R<deg>` | move both, coordinated so they finish together |
| `J<p> <r>` | jog rates in deg/s (what the controller script sends ~50×/s) |
| `Z` | set current pose as zero |
| `V<deg/s>` | speed for P/R moves |
| `A<deg/s²>` | acceleration for moves and jogging |
| `E` / `D` | enable / disable drivers |
| `X` | stop (decelerate) |
| `?` | status |

## Configuration

**Firmware** (`diff_wrist_xbox.ino`, CONFIG section):

| Setting | What it is |
|---|---|
| `MICROSTEPS` | must match the R208 SR1/SR2 jumpers |
| `PITCH_MIN/MAX`, `ROLL_MIN/MAX` | soft joint limits in degrees. Tighten these to what your build can physically reach |
| `jointSpeed`, `jointAccel` | default speed and acceleration |
| `JOG_RATE_MAX` | highest jog rate the firmware accepts, in deg/s |
| `INVERT_A`, `INVERT_B`, `SWAP_PITCH_ROLL` | fix reversed or swapped axes here, not in the math |

**Mac script** (top of `xbox_wrist.py`):

| Setting | What it is |
|---|---|
| `AXIS_*`, `BTN_*` | which stick/button does what (find numbers with `--show`) |
| `INVERT_PITCH`, `INVERT_ROLL` | flip stick direction |
| `DEADZONE`, `EXPO` | stick feel |
| `RATE_STEPS` | speeds RB/LB step through. Values above the firmware's `JOG_RATE_MAX` are capped |

## Troubleshooting

**`--show` finds the controller but all axes stay at 0.00**
- Connect over Bluetooth instead of USB.
- Make sure no other copy of the script is running (`pgrep -fl xbox_wrist.py`). On macOS,
  one copy can hold the controller and starve the other. Stop strays with
  `pkill -f xbox_wrist.py`.
- The script sets `SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=1` so input reaches Python even
  while Terminal has focus. Keep that line above `import pygame`.

**"No controller found"**: the controller went to sleep. Press the Xbox button until the
logo is solid and run again. The script only looks for a controller at startup.

**"No /dev/cu.usbmodem\* found"**: the Arduino isn't plugged in, or the Serial Monitor has
the port open.

**Only one motor moves**: wiring or driver problem, not the code. Check that driver's
STEP/DIR pins, Opto Supply 5V, +V, and current pot, or swap the two drivers to see if the
fault follows the driver. `basic_stepper_test.ino` drives motor A only (pins 2/3) and
moves many turns, so run it with the gears **uncoupled**.

**Rough or noisy motion at low speed**: low-speed stepper resonance. Finer microstepping
(change jumpers **and** `MICROSTEPS`) smooths it out.

**Axes reversed or swapped**: use `INVERT_A`, `INVERT_B`, `SWAP_PITCH_ROLL` in the
firmware, or `INVERT_PITCH`/`INVERT_ROLL` in the script.

## Known limitations

- Joint limits are guesses; nothing in the design defines them.
- Switching from jogging to a `P`/`R` move while still moving can jolt. Fine when stopped.
- The printed motor plates and pinion are designed for NEMA 17. NEMA 23 needs a different
  bolt pattern (47.14 mm), pilot (38.1 mm), and shaft bore (6.35 or 8 mm).

## License

Mechanical design © tiago, CC BY-SA 4.0. Derivatives of the design must be shared under the
same license with credit.
