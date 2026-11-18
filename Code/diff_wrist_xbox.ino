/*
  Differential wrist controller — 2x NEMA 23 + 2x Lin Engineering R208
  Board: Arduino UNO R4 (Minima or WiFi). Library: AccelStepper (Library Manager).
  v1 + Xbox controller: run xbox_wrist.py on the Mac, it sends J commands over USB.

  Built for 3DPDiffWrist by tiago (Printables, CC BY-SA 4.0). Per the PDF:
    Big gears turn the same direction -> ROLL
    Big gears turn opposite           -> PITCH (top gear pivots on the cross-pin)
  So motor angles are a sum/difference of the joint angles (same as GRBL CoreXY):
    motorA = G_ROLL*roll + G_PITCH*pitch
    motorB = G_ROLL*roll - G_PITCH*pitch

  Serial commands (115200 baud, newline). You can still type these in the Serial Monitor:
    P<deg>          move pitch to an angle, e.g.  P30
    R<deg>          move roll to an angle,  e.g.  R-45
    P<deg> R<deg>   move both at once (coordinated, both motors finish together)
    J<p> <r>        JOG: pitch rate and roll rate in deg/s, e.g. J20 -10
                    (the controller script sends this ~50x/s; if they stop
                     arriving for 300 ms the wrist ramps to a stop: dead-man)
    Z               set the current pose as zero (pitch=0, roll=0)
    V<deg/s>        speed limit for P/R moves
    A<deg/s^2>      acceleration (used by both P/R moves and jogging)
    E / D           enable / disable drivers
    ?               print status
    X               stop (decelerates)
*/

#include <AccelStepper.h>

// ================================ CONFIG ======================================

// Pins (STEP/DIR to the R208 "Step" and "Dir" inputs; Opto Supply to 5V)
const uint8_t A_STEP = 2, A_DIR = 3;
const uint8_t B_STEP = 4, B_DIR = 5;
const int8_t  DISABLE_PIN = -1;           // wire to both R208 "Disable" inputs; -1 if unused
const bool    DISABLE_ACTIVE_HIGH = true; // flip if the drivers turn OFF when you send E

// Motor/driver
const float FULL_STEPS_PER_REV = 200.0;   // 1.8° motor. Leave at 200 whatever the microstepping.
const float MICROSTEPS         = 1.0;     // must match the R208's SR1/SR2 setting (2 = half step)

// Gearing from the 3DPDiffWrist PDF:
//   smallgear (motor pinion) = 11T, biggear (side bevel) = 53T, topgear (output) = 53T
const float PINION_TEETH  = 11;
const float BIGGEAR_TEETH = 53;
const float TOPGEAR_TEETH = 53;
const float INPUT_REDUCTION = BIGGEAR_TEETH / PINION_TEETH;   // 4.818 : 1
const float BEVEL_RATIO     = BIGGEAR_TEETH / TOPGEAR_TEETH;  // 1 : 1
// -> half step: 200 * 2 * 53 / 11 / 360 = 5.35 motor steps per joint degree

// Direction fixes (set after the first test)
const bool INVERT_A = false;
const bool INVERT_B = true;   // the motors face each other, so one is mirrored
const bool SWAP_PITCH_ROLL = false;

// Joint limits (degrees). Guesses: jog slowly and tighten these to your build.
const float PITCH_MIN = -360, PITCH_MAX = 360;
const float ROLL_MIN  = -360, ROLL_MAX  = 360;

// Motion
float jointSpeed = 400;    // deg/s for P/R moves
float jointAccel = 800;   // deg/s^2 for moves and jogging
const float JOG_RATE_MAX = 400;              // deg/s cap on J commands
const unsigned long DEADMAN_MS = 300;        // no J for this long -> ramp to stop

// ==============================================================================

const float MOTOR_STEPS_PER_DEG = FULL_STEPS_PER_REV * MICROSTEPS / 360.0;
const float G_ROLL  = INPUT_REDUCTION * MOTOR_STEPS_PER_DEG;               // steps per roll deg
const float G_PITCH = INPUT_REDUCTION / BEVEL_RATIO * MOTOR_STEPS_PER_DEG; // steps per pitch deg

AccelStepper motA(AccelStepper::DRIVER, A_STEP, A_DIR);
AccelStepper motB(AccelStepper::DRIVER, B_STEP, B_DIR);

float targetPitch = 0, targetRoll = 0;
bool  enabled = false;

// Jog (velocity) mode state
bool  jogMode = false;
float jogCmdP = 0, jogCmdR = 0;   // commanded joint rates from the controller (deg/s)
float jogP = 0, jogR = 0;         // actual joint rates after ramping (deg/s)
unsigned long lastJogMs = 0, lastRampUs = 0;

// ---------- kinematics ----------
void jointsToSteps(float pitch, float roll, long &a, long &b) {
  if (SWAP_PITCH_ROLL) { float t = pitch; pitch = roll; roll = t; }
  a = lround(G_ROLL * roll + G_PITCH * pitch);
  b = lround(G_ROLL * roll - G_PITCH * pitch);
}

void stepsToJoints(long a, long b, float &pitch, float &roll) {
  roll  = (a + b) / (2.0 * G_ROLL);
  pitch = (a - b) / (2.0 * G_PITCH);
  if (SWAP_PITCH_ROLL) { float t = pitch; pitch = roll; roll = t; }
}

// Same math for speeds (deg/s <-> steps/s); it's linear, so it's identical.
void jointRatesToStepRates(float vp, float vr, float &sa, float &sb) {
  if (SWAP_PITCH_ROLL) { float t = vp; vp = vr; vr = t; }
  sa = G_ROLL * vr + G_PITCH * vp;
  sb = G_ROLL * vr - G_PITCH * vp;
}

// ---------- drivers ----------
void setEnabled(bool on) {
  enabled = on;
  if (DISABLE_PIN >= 0) {
    bool disableLevel = DISABLE_ACTIVE_HIGH ? HIGH : LOW;
    digitalWrite(DISABLE_PIN, on ? !disableLevel : disableLevel);
  }
}

// ---------- position moves (v1) ----------
// Coordinated move: scale each motor's speed/accel by its share of the distance
// so both trapezoids have the same duration and the wrist follows a straight joint-space path.
void moveTo(float pitch, float roll) {
  jogMode = false;
  pitch = constrain(pitch, PITCH_MIN, PITCH_MAX);
  roll  = constrain(roll,  ROLL_MIN,  ROLL_MAX);
  targetPitch = pitch; targetRoll = roll;

  long a, b;
  jointsToSteps(pitch, roll, a, b);
  long dA = labs(a - motA.currentPosition());
  long dB = labs(b - motB.currentPosition());
  long dMax = max(dA, dB);
  if (dMax == 0) return;

  float gMax = max(G_PITCH, G_ROLL);
  float vMax = jointSpeed * gMax;   // steps/s
  float aMax = jointAccel * gMax;   // steps/s^2

  float fA = (float)dA / dMax, fB = (float)dB / dMax;
  motA.setMaxSpeed(max(vMax * fA, 1.0f));
  motA.setAcceleration(max(aMax * fA, 1.0f));
  motB.setMaxSpeed(max(vMax * fB, 1.0f));
  motB.setAcceleration(max(aMax * fB, 1.0f));

  if (!enabled) setEnabled(true);
  motA.moveTo(a);
  motB.moveTo(b);
}

// ---------- jog mode (controller) ----------
void startJog(float p, float r) {
  if (!jogMode) {
    // a P/R move might be in progress: hand off without a jolt by
    // reading the motors' current speeds back into joint rates
    float sa = motA.speed(), sb = motB.speed();
    jogR = (sa + sb) / (2.0 * G_ROLL);
    jogP = (sa - sb) / (2.0 * G_PITCH);
    if (SWAP_PITCH_ROLL) { float t = jogP; jogP = jogR; jogR = t; }
    lastRampUs = micros();
    jogMode = true;
  }
  jogCmdP = constrain(p, -JOG_RATE_MAX, JOG_RATE_MAX);
  jogCmdR = constrain(r, -JOG_RATE_MAX, JOG_RATE_MAX);
  lastJogMs = millis();
  if (!enabled) setEnabled(true);
}

// Fastest you can be going and still stop before the limit (v^2 = 2 a d)
float limitRate(float rate, float pos, float lo, float hi) {
  if (rate > 0) return min(rate,  sqrtf(2 * jointAccel * max(hi - pos, 0.0f)));
  if (rate < 0) return max(rate, -sqrtf(2 * jointAccel * max(pos - lo, 0.0f)));
  return 0;
}

// Runs every 5 ms in jog mode: ramp actual rates toward the command, respect limits,
// then hand the resulting motor step rates to AccelStepper.
void updateJog() {
  unsigned long now = micros();
  float dt = (now - lastRampUs) * 1e-6;
  if (dt < 0.005) return;
  lastRampUs = now;

  if (millis() - lastJogMs > DEADMAN_MS) { jogCmdP = 0; jogCmdR = 0; }   // dead-man

  float pos_p, pos_r;
  stepsToJoints(motA.currentPosition(), motB.currentPosition(), pos_p, pos_r);
  float wantP = limitRate(jogCmdP, pos_p, PITCH_MIN, PITCH_MAX);
  float wantR = limitRate(jogCmdR, pos_r, ROLL_MIN,  ROLL_MAX);

  float dv = jointAccel * dt;   // max change in rate this tick
  jogP += constrain(wantP - jogP, -dv, dv);
  jogR += constrain(wantR - jogR, -dv, dv);

  float sa, sb;
  jointRatesToStepRates(jogP, jogR, sa, sb);
  motA.setMaxSpeed(max(fabsf(sa), 1.0f));   // setSpeed is clamped to maxSpeed
  motB.setMaxSpeed(max(fabsf(sb), 1.0f));
  motA.setSpeed(sa);
  motB.setSpeed(sb);

  targetPitch = pos_p; targetRoll = pos_r;  // keep v1 P/R moves relative to where we are
}

// ---------- status ----------
void printStatus() {
  float p, r;
  stepsToJoints(motA.currentPosition(), motB.currentPosition(), p, r);
  Serial.print(F("pitch=")); Serial.print(p, 2);
  Serial.print(F("  roll="));  Serial.print(r, 2);
  Serial.print(jogMode ? F("  | JOG ") : F("  | target "));
  if (jogMode) { Serial.print(jogP, 1); Serial.print('/'); Serial.print(jogR, 1); Serial.print(F(" deg/s")); }
  else         { Serial.print(targetPitch, 1); Serial.print('/'); Serial.print(targetRoll, 1); }
  Serial.print(F("  | steps A=")); Serial.print(motA.currentPosition());
  Serial.print(F(" B="));          Serial.print(motB.currentPosition());
  Serial.print(F("  | V="));  Serial.print(jointSpeed);
  Serial.print(F(" A="));     Serial.print(jointAccel);
  Serial.println(enabled ? F("  [ENABLED]") : F("  [disabled]"));
}

void stopAll() {
  if (jogMode) { jogCmdP = jogCmdR = 0; lastJogMs = 0; }   // ramps down via updateJog
  else         { motA.stop(); motB.stop(); }
}

// ---------- serial parsing ----------
void handleLine(String s) {
  s.trim(); s.toUpperCase();
  if (s.length() == 0) return;

  // J<p> <r>: jog rates. Parsed separately because it has two bare numbers.
  if (s[0] == 'J') {
    String rest = s.substring(1); rest.trim();
    int sp = rest.indexOf(' ');
    float p = rest.toFloat();
    float r = (sp >= 0) ? rest.substring(sp + 1).toFloat() : 0;
    startJog(p, r);
    return;   // no reply: this arrives 50x a second
  }

  bool havePitch = false, haveRoll = false;
  float p = targetPitch, r = targetRoll;

  for (int i = 0; i < (int)s.length(); i++) {
    char c = s[i];
    float val = s.substring(i + 1).toFloat();
    switch (c) {
      case 'P': p = val; havePitch = true; break;
      case 'R': r = val; haveRoll  = true; break;
      case 'V': if (val > 0) jointSpeed = val; Serial.println(F("speed set")); break;
      case 'A': if (val > 0) jointAccel = val; Serial.println(F("accel set")); break;
      case 'Z': motA.setCurrentPosition(0); motB.setCurrentPosition(0);
                jogP = jogR = 0;
                targetPitch = targetRoll = 0; Serial.println(F("zeroed")); break;
      case 'E': setEnabled(true);  Serial.println(F("enabled"));  break;
      case 'D': stopAll(); setEnabled(false); Serial.println(F("disabled")); break;
      case 'X': stopAll(); Serial.println(F("stopping")); break;
      case '?': printStatus(); break;
      default: break;  // digits, '-', '.', spaces belong to a value
    }
  }
  if (havePitch || haveRoll) {
    moveTo(p, r);
    Serial.print(F("-> pitch ")); Serial.print(targetPitch);
    Serial.print(F("  roll "));   Serial.println(targetRoll);
  }
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  if (DISABLE_PIN >= 0) pinMode(DISABLE_PIN, OUTPUT);
  setEnabled(false);

  motA.setPinsInverted(INVERT_A, false, false);
  motB.setPinsInverted(INVERT_B, false, false);
  motA.setMinPulseWidth(5);   // give the R208's opto inputs a clean pulse
  motB.setMinPulseWidth(5);

  delay(500);
  Serial.println(F("Diff wrist ready. Put the wrist at its neutral pose, then send Z."));
  Serial.println(F("Try: P20   R45   P0 R0   ?   (or run xbox_wrist.py)"));
}

String buf;
void loop() {
  if (jogMode) {
    updateJog();
    motA.runSpeed();   // constant-speed stepping at whatever updateJog set
    motB.runSpeed();
  } else {
    motA.run();        // v1 accelerated position moves
    motB.run();
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { handleLine(buf); buf = ""; }
    else buf += c;
  }
}
