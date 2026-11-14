#include <Arduino.h>
#include <math.h>
#include <arm_kinematics.h>
#include <stdlib.h>
#include <stdio.h>

// TWO UDK2120A Drivers (to update) + TWO PK266MA Stepper Motors
// Arduino UNO R3

// Each stepper turns the lead screw of a linear actuator that swings one
// joint of the arm in Drawing 1:
//   Motor 1: shoulder actuator (base -> link 1)
//   Motor 2: elbow actuator    (link 1 -> forearm)

// Motor 1: Shoulder
const uint8_t M1_PLS   = 2;
const uint8_t M1_DIR   = 3;
const uint8_t M1_COFF  = 4;
const uint8_t M1_OHEAT = 5;

// Motor 2: Elbow
const uint8_t M2_PLS   = 6;
const uint8_t M2_DIR   = 7;
const uint8_t M2_COFF  = 8;
const uint8_t M2_OHEAT = 9;

// Update this against the driver switch table when switching
const long STEPS_PER_REV = 400;

// TODO: actuator data and bounds
// get the pitch ffs
constexpr float ACT1_MM_PER_REV = 0.0f;
constexpr float ACT1_MIN_LEN_MM = 0.0f;
constexpr float ACT1_MAX_LEN_MM = 0.0f;

constexpr float ACT2_MM_PER_REV = 0.0f;
constexpr float ACT2_MIN_LEN_MM = 0.0f;
constexpr float ACT2_MAX_LEN_MM = 0.0f;

// DO NOT RUN THE CODE WITHOUT FILLING THE VALUES ABOVE
// SERIOUSLY DO NOT RUN IT WILL GET UGLY

const bool M1_EXTEND_HIGH = true;
const bool M2_EXTEND_HIGH = true;

float maxSpeed = 8000.0;
float accel = 16000.0;

const float START_SPEED = 200.0;
const float SPEED_LIMIT = 14000.0;

const unsigned int PULSE_ON_US = 10;
const unsigned int DIR_SETUP_US = 50;

// Motor Data
arm::Geometry geo;

struct Motor {
  uint8_t pls;
  uint8_t dir;
  uint8_t coff;
  uint8_t oheat;

  long position;     // steps from the reference pose; positive = actuator lengthens
  bool dirLevel;     // level last driven on DIR
  bool enabled;

  bool extendHigh;
  float stepsPerMm;
  float minLen;
  float maxLen;
  float refLen;      // pin-to-pin length at the initial (reference) pose
};

Motor motor1 = {
  M1_PLS, M1_DIR, M1_COFF, M1_OHEAT,
  0, false, true,
  M1_EXTEND_HIGH,
  STEPS_PER_REV / ACT1_MM_PER_REV, ACT1_MIN_LEN_MM, ACT1_MAX_LEN_MM, 0.0
};

Motor motor2 = {
  M2_PLS, M2_DIR, M2_COFF, M2_OHEAT,
  0, false, true,
  M2_EXTEND_HIGH,
  STEPS_PER_REV / ACT2_MM_PER_REV, ACT2_MIN_LEN_MM, ACT2_MAX_LEN_MM, 0.0
};

// Low-Level Control Things

// Both pulses share one pulse width so that a coordinated move spends the
// same time per tick as a single-axis move.
void pulseAxes(bool p1, bool p2) {
  if (p1) digitalWrite(motor1.pls, HIGH);
  if (p2) digitalWrite(motor2.pls, HIGH);
  delayMicroseconds(PULSE_ON_US);
  if (p1) digitalWrite(motor1.pls, LOW);
  if (p2) digitalWrite(motor2.pls, LOW);
}

void setDirection(Motor &m, bool extend) {
  bool level = extend ? m.extendHigh : !m.extendHigh;
  if (level == m.dirLevel) return;

  digitalWrite(m.dir, level ? HIGH : LOW);
  m.dirLevel = level;

  delayMicroseconds(DIR_SETUP_US);
}

bool overheated(Motor &m) {
  // Is it N.O. O.HEAT wiring with INPUT_PULLUP?
  return digitalRead(m.oheat) == LOW;
}

void enableMotor(Motor &m, bool on) {
  digitalWrite(m.coff, on ? LOW : HIGH);
  m.enabled = on;

  if (on) delay(100);

  Serial.println(on ? F("Motor enabled")
                    : F("Motor free"));
}

// Position <--> actuator length

float lengthAt(const Motor &m, long pos) {
  return m.refLen + (float)pos / m.stepsPerMm;
}

long stepsForLength(const Motor &m, float len) {
  return lround((len - m.refLen) * m.stepsPerMm);
}

// Stop a move before it starts to avoid desync
bool withinTravel(const Motor &m, long pos, uint8_t id) {
  float len = lengthAt(m, pos);
  if (len >= m.minLen && len <= m.maxLen) return true;

  Serial.print(F("Motor "));
  Serial.print(id);
  Serial.print(F(" would reach "));
  Serial.print(len);
  Serial.print(F(" mm; allowed "));
  Serial.print(m.minLen);
  Serial.print(F(" to "));
  Serial.println(m.maxLen);
  return false;
}

// Movement
// Positive steps = CW; negative steps = CCW

bool moveAxes(long s1, long s2) {
  if (s1 == 0 && s2 == 0) return true;

  Motor *mot[2] = {&motor1, &motor2};
  long steps[2] = {s1, s2};

  for (uint8_t k = 0; k < 2; k++) {
    if (steps[k] == 0) continue;

    if (!mot[k]->enabled) {
      Serial.println(F("Motor free"));
      return false;
    }

    if (overheated(*mot[k])) {
      Serial.println(F("OVERHEAT - do not move"));
      return false;
    }

    if (!withinTravel(*mot[k], mot[k]->position + steps[k], k + 1)) {
      return false;
    }
  }

  if (s1 != 0) setDirection(motor1, s1 > 0);
  if (s2 != 0) setDirection(motor2, s2 > 0);

  long a1 = labs(s1);
  long a2 = labs(s2);
  long n = (a1 > a2) ? a1 : a2;

  float vmax = constrain(
    maxSpeed, START_SPEED, SPEED_LIMIT
  );

  float safeAccel = (accel < 1.0) ? 1.0 : accel;

  long ramp = (long)(
    (vmax * vmax - START_SPEED * START_SPEED)
    / (2.0 * safeAccel)
  );

  if (ramp > n / 2) ramp = n / 2;

  long err1 = n / 2;
  long err2 = n / 2;
  long done1 = 0;
  long done2 = 0;
  bool aborted = false;

  unsigned long t = micros();

  for (long i = 0; i < n; i++) {
    long fromEnd =
      (i < n - 1 - i) ? i : (n - 1 - i);

    float v;

    if (fromEnd < ramp) {
      v = sqrt(
        START_SPEED * START_SPEED
        + 2.0 * safeAccel * fromEnd
      );
    } else {
      v = vmax;
    }

    if (v > vmax) v = vmax;

    err1 += a1;
    err2 += a2;
    bool p1 = err1 >= n;
    bool p2 = err2 >= n;
    if (p1) { err1 -= n; done1++; }
    if (p2) { err2 -= n; done2++; }

    pulseAxes(p1, p2);

    t += (unsigned long)(1000000.0 / v);

    while ((long)(micros() - t) < 0) {
      // Wait for the next step interval.
    }

    if ((i & 0x3F) == 0 &&
        ((a1 > 0 && overheated(motor1)) ||
         (a2 > 0 && overheated(motor2)))) {
      Serial.println(F("OVERHEAT - aborted"));
      aborted = true;
      break;
    }
  }

  motor1.position += (s1 > 0) ? done1 : -done1;
  motor2.position += (s2 > 0) ? done2 : -done2;

  Serial.print(F("Position = "));
  Serial.print(motor1.position);
  Serial.print(F(", "));
  Serial.println(motor2.position);

  return !aborted;
}

bool moveSteps(Motor &m, long steps) {
  if (&m == &motor1) return moveAxes(steps, 0);
  return moveAxes(0, steps);
}

// Arm-level control

float shoulderDeg() {
  return arm::toDeg(
    geo.shoulder.angleAt(lengthAt(motor1, motor1.position)));
}

float elbowDeg() {
  return arm::toDeg(
    geo.elbow.angleAt(lengthAt(motor2, motor2.position)));
}

long shoulderStepsFor(float deg) {
  return stepsForLength(
    motor1, geo.shoulder.lengthAt(arm::toRad(deg)));
}

long elbowStepsFor(float deg) {
  return stepsForLength(
    motor2, geo.elbow.lengthAt(arm::toRad(deg)));
}

bool moveJoints(float shoulder, float elbow) {
  long t1 = shoulderStepsFor(shoulder);
  long t2 = elbowStepsFor(elbow);
  return moveAxes(t1 - motor1.position, t2 - motor2.position);
}

bool moveTip(float reach, float height) {
  float t1, t2;

  if (!arm::anglesFromTip(geo, reach, height, t1, t2)) {
    Serial.println(F("Unreachable: outside the arm's link lengths"));
    return false;
  }

  return moveJoints(arm::toDeg(t1), arm::toDeg(t2));
}

void printStatus() {
  arm::Vec tip =
    arm::tipFromAngles(geo, arm::toRad(shoulderDeg()),
                       arm::toRad(elbowDeg()));

  Serial.print(F("Steps:    "));
  Serial.print(motor1.position);
  Serial.print(F(", "));
  Serial.println(motor2.position);

  Serial.print(F("Length:   "));
  Serial.print(lengthAt(motor1, motor1.position));
  Serial.print(F(", "));
  Serial.print(lengthAt(motor2, motor2.position));
  Serial.println(F(" mm"));

  Serial.print(F("Angles:   "));
  Serial.print(shoulderDeg());
  Serial.print(F(", "));
  Serial.print(elbowDeg());
  Serial.println(F(" deg"));

  Serial.print(F("Tip:      reach "));
  Serial.print(tip.x);
  Serial.print(F(", height "));
  Serial.print(tip.y);
  Serial.println(F(" mm"));
}

// Serial Command Help Print

void printHelp() {
  Serial.println();
  Serial.println(F("=== ARM CONTROLLER ==="));
  Serial.println(F("j <sh> <el>   Joint angles, deg (0,0 = drawn pose)"));
  Serial.println(F("g <x> <y>     Tip reach/height, mm from shoulder pivot"));
  Serial.println(F("m1 <steps>    Move shoulder actuator"));
  Serial.println(F("m2 <steps>    Move elbow actuator"));
  Serial.println(F("r1 <revs>     Rotate motor 1"));
  Serial.println(F("r2 <revs>     Rotate motor 2"));
  Serial.println(F("b <s1> <s2>   Move both together, in steps"));
  Serial.println(F("v <steps/s>   Set maximum speed"));
  Serial.println(F("a <steps/s2>  Set acceleration"));
  Serial.println(F("e             Enable both motors"));
  Serial.println(F("f             Free both (arm can drop if screws backdrive)"));
  Serial.println(F("p             Print position, length, angles, tip"));
  Serial.println(F("z [sh el]     Declare current pose (default: CAD pose)"));
  Serial.println(F("================================"));
  Serial.print(F("Steps/rev: "));
  Serial.println(STEPS_PER_REV);
  Serial.print(F("Max speed: "));
  Serial.println(maxSpeed);
  Serial.print(F("Acceleration: "));
  Serial.println(accel);
}

// Setup

void setup() {
  digitalWrite(M1_PLS, LOW);
  pinMode(M1_PLS, OUTPUT);

  digitalWrite(M1_DIR, LOW);
  pinMode(M1_DIR, OUTPUT);

  digitalWrite(M1_COFF, LOW);
  pinMode(M1_COFF, OUTPUT);

  digitalWrite(M2_PLS, LOW);
  pinMode(M2_PLS, OUTPUT);

  digitalWrite(M2_DIR, LOW);
  pinMode(M2_DIR, OUTPUT);

  digitalWrite(M2_COFF, LOW);
  pinMode(M2_COFF, OUTPUT);

  pinMode(M1_OHEAT, INPUT_PULLUP);
  pinMode(M2_OHEAT, INPUT_PULLUP);

  motor1.dirLevel = false;
  motor2.dirLevel = false;

  geo = arm::buildGeometry();
  motor1.refLen = geo.shoulder.lengthAt(0.0f);
  motor2.refLen = geo.elbow.lengthAt(0.0f);

  Serial.begin(115200);
  Serial.setTimeout(100);

  delay(100);

  Serial.println(F("Arm controller ready"));
  Serial.print(F("Drawn-pose actuator lengths (mm): "));
  Serial.print(motor1.refLen);
  Serial.print(F(", "));
  Serial.println(motor2.refLen);
  printHelp();
}

// Loop and Man. Control

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.length() == 0) return;

  int spaceIndex = line.indexOf(' ');

  String cmd;
  String args;

  if (spaceIndex < 0) {
    cmd = line;
    args = "";
  } else {
    cmd = line.substring(0, spaceIndex);
    args = line.substring(spaceIndex + 1);
  }

  cmd.toLowerCase();

  float x = 0;
  float y = 0;

  // Read up to two numeric arguments.
  // sscanf i hate you
  int secondSpace = args.indexOf(' ');

  if (secondSpace >= 0) {
    x = args.substring(0, secondSpace).toFloat();
    y = args.substring(secondSpace + 1).toFloat();
  } else {
    x = args.toFloat();
  }

  if (cmd == "m1") {
    moveSteps(motor1, (long)x);
  }
  else if (cmd == "m2") {
    moveSteps(motor2, (long)x);
  }
  else if (cmd == "r1") {
    moveSteps(
      motor1,
      (long)round(x * STEPS_PER_REV)
    );
  }
  else if (cmd == "r2") {
    moveSteps(
      motor2,
      (long)round(x * STEPS_PER_REV)
    );
  }
  else if (cmd == "b") {
    moveAxes((long)x, (long)y);
  }
  else if (cmd == "j") {
    moveJoints(x, y);
  }
  else if (cmd == "g") {
    moveTip(x, y);
  }
  else if (cmd == "v") {
    maxSpeed = constrain(
      x, START_SPEED, SPEED_LIMIT
    );

    Serial.print(F("Max speed = "));
    Serial.println(maxSpeed);
  }
  else if (cmd == "a") {
    accel = (x < 1) ? 1 : x;

    Serial.print(F("Acceleration = "));
    Serial.println(accel);
  }
  else if (cmd == "e") {
    enableMotor(motor1, true);
    enableMotor(motor2, true);
  }
  else if (cmd == "f") {
    enableMotor(motor1, false);
    enableMotor(motor2, false);
  }
  else if (cmd == "p") {
    printStatus();
  }
  else if (cmd == "z") {
    if (args.length() == 0) {
      x = 0;
      y = 0;
    }

    motor1.position = shoulderStepsFor(x);
    motor2.position = elbowStepsFor(y);
    Serial.println(F("Pose declared"));
    printStatus();
  }
  else {
    printHelp();
  }
}