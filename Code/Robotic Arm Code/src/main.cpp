#include <Arduino.h>
#include <ExcelStepper.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// Two UDK2120A drivers + two PK266MA steppers, Arduino UNO R3
// This variant uses ExcelStepper and FastPin libraries

// Motor 1
const uint8_t M1_PLS   = 2;
const uint8_t M1_DIR   = 3;
const uint8_t M1_COFF  = 4;
const uint8_t M1_OHEAT = 5;

// Motor 2
const uint8_t M2_PLS   = 6;
const uint8_t M2_DIR   = 7;
const uint8_t M2_COFF  = 8;
const uint8_t M2_OHEAT = 9;

// Pulses per motor revolution
const long STEPS_PER_REV = 400;

// Flip this if a motor is moving wrong
const bool M1_INVERT = false;
const bool M2_INVERT = false;

// Curve things
const uint16_t START_SPEED = 200;
const uint16_t SPEED_LIMIT = 8000;
uint16_t cruiseSpeed = 2000;

// Rate of acceleration
const long RAMP_STEPS = 200;

const unsigned int DIR_SETUP_US = 50;
const unsigned int PULSE_ON_US = 10;

// Axis
// Motors can move together or on their own
enum Phase { IDLE, ACCEL, CRUISE, DECEL };

struct Axis {
  ExcelStepper stepper;
  uint8_t id;
  uint8_t pls;
  uint8_t coff;
  uint8_t oheat;
  bool invert;
  bool enabled;

  Phase phase;
  long ramp;
  long cruise;

  Axis(uint8_t id_, uint8_t pls_, uint8_t dir, uint8_t coff_, uint8_t oheat_,
       bool invert_)
    : stepper(pls_, dir, START_SPEED),
      id(id_), pls(pls_), coff(coff_), oheat(oheat_), invert(invert_),
      enabled(true), phase(IDLE), ramp(0), cruise(0) {}
};

Axis axes[2] = {
  Axis(1, M1_PLS, M1_DIR, M1_COFF, M1_OHEAT, M1_INVERT),
  Axis(2, M2_PLS, M2_DIR, M2_COFF, M2_OHEAT, M2_INVERT)
};

bool overheated(Axis &a) {
  // Is it N.O. O.HEAT wiring with INPUT_PULLUP?
  return digitalRead(a.oheat) == LOW;
}

void releaseStepPin(Axis &a) {
  delayMicroseconds(PULSE_ON_US);
  digitalWrite(a.pls, LOW);
}

void say(Axis &a, const __FlashStringHelper *msg) {
  Serial.print(F("Motor "));
  Serial.print((int)a.id);
  Serial.print(F(" "));
  Serial.println(msg);
}

void finishMove(Axis &a) {
  a.phase = IDLE;
  releaseStepPin(a);
  say(a, F("done"));
}

void stopAxis(Axis &a) {
  if (a.phase == IDLE) return;

  a.stepper.jumpToSpeed(0);
  a.phase = IDLE;
  releaseStepPin(a);
}

bool startMove(Axis &a, long steps) {
  if (steps == 0) return false;

  if (!a.enabled) {
    say(a, F("free"));
    return false;
  }

  if (a.phase != IDLE) {
    say(a, F("busy"));
    return false;
  }

  if (overheated(a)) {
    Serial.println(F("OVERHEAT - do not move"));
    return false;
  }

  bool forward = (steps > 0) != a.invert;
  a.stepper.setDirection(
    forward ? ExcelStepper::CLOCKWISE : ExcelStepper::COUNTERCLOCKWISE
  );
  delayMicroseconds(DIR_SETUP_US);

  long n = labs(steps);
  a.ramp = (n / 2 < RAMP_STEPS) ? n / 2 : RAMP_STEPS;
  a.cruise = n - 2 * a.ramp;

  if (a.ramp > 0) {
    a.stepper.accelerate(cruiseSpeed, a.ramp);
    a.phase = ACCEL;
  } else {
    // ExcelStepper only steps once a non-stop speed is set, so start speed is set.
    a.stepper.jumpToSpeed(START_SPEED);
    a.stepper.move(a.cruise);
    a.phase = CRUISE;
  }

  return true;
}

// Called as often as possible because ExcelStepper only steps inside run().
void serviceAxis(Axis &a) {
  if (a.phase == IDLE) return;

  if (overheated(a)) {
    Serial.println(F("OVERHEAT - stopped"));
    stopAxis(a);
    return;
  }

  if (!a.stepper.run()) return;

  switch (a.phase) {
    case ACCEL:
      if (a.cruise > 0) {
        a.stepper.move(a.cruise);
        a.phase = CRUISE;
      } else {
        a.stepper.decelerate(0, a.ramp);
        a.phase = DECEL;
      }
      break;

    case CRUISE:
      if (a.ramp > 0) {
        a.stepper.decelerate(0, a.ramp);
        a.phase = DECEL;
      } else {
        a.stepper.jumpToSpeed(0);
        finishMove(a);
      }
      break;

    case DECEL:
      finishMove(a);
      break;

    default:
      break;
  }
}

void enableAxis(Axis &a, bool on) {
  if (!on) stopAxis(a);

  digitalWrite(a.coff, on ? LOW : HIGH);
  a.enabled = on;
}

void printHelp() {
  Serial.println();
  Serial.println(F("=== TWO MOTOR CONTROLLER ==="));
  Serial.println(F("m1 <steps>    Move motor 1 (+/- relative)"));
  Serial.println(F("m2 <steps>    Move motor 2 (+/- relative)"));
  Serial.println(F("r1 <revs>     Rotate motor 1"));
  Serial.println(F("r2 <revs>     Rotate motor 2"));
  Serial.println(F("b <s1> <s2>   Move both at once"));
  Serial.println(F("x             Stop now"));
  Serial.println(F("v <steps/s>   Set cruise speed"));
  Serial.println(F("e / f         Enable / free both motors"));
  Serial.println(F("================================"));
  Serial.print(F("Cruise speed: "));
  Serial.println((long)cruiseSpeed);
}

// Command handling

void handleLine(char *line) {
  char *cmd = strtok(line, " ");
  if (!cmd) return;
  for (char *p = cmd; *p; p++) *p = tolower(*p);

  char *a1 = strtok(NULL, " ");
  char *a2 = strtok(NULL, " ");

  long x = a1 ? atol(a1) : 0;
  long y = a2 ? atol(a2) : 0;
  float f = a1 ? atof(a1) : 0.0;

  if (!strcmp(cmd, "m1")) {
    startMove(axes[0], x);
  }
  else if (!strcmp(cmd, "m2")) {
    startMove(axes[1], x);
  }
  else if (!strcmp(cmd, "r1")) {
    startMove(axes[0], lround(f * STEPS_PER_REV));
  }
  else if (!strcmp(cmd, "r2")) {
    startMove(axes[1], lround(f * STEPS_PER_REV));
  }
  else if (!strcmp(cmd, "b")) {
    startMove(axes[0], x);
    startMove(axes[1], y);
  }
  else if (!strcmp(cmd, "x")) {
    stopAxis(axes[0]);
    stopAxis(axes[1]);
    Serial.println(F("Stopped"));
  }
  else if (!strcmp(cmd, "v")) {
    long v = x;
    if (v < START_SPEED) v = START_SPEED;
    if (v > SPEED_LIMIT) v = SPEED_LIMIT;
    cruiseSpeed = v;
    Serial.print(F("Cruise speed = "));
    Serial.println(v);
  }
  else if (!strcmp(cmd, "e")) {
    enableAxis(axes[0], true);
    enableAxis(axes[1], true);
    delay(100);
    Serial.println(F("Motors enabled"));
  }
  else if (!strcmp(cmd, "f")) {
    enableAxis(axes[0], false);
    enableAxis(axes[1], false);
    Serial.println(F("Motors free"));
  }
  else {
    printHelp();
  }
}

char lineBuf[40];
uint8_t lineLen = 0;

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        lineLen = 0;
        handleLine(lineBuf);
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

void setup() {
  for (uint8_t i = 0; i < 2; i++) {
    axes[i].stepper.begin();

    digitalWrite(axes[i].coff, LOW);
    pinMode(axes[i].coff, OUTPUT);
    pinMode(axes[i].oheat, INPUT_PULLUP);
  }

  Serial.begin(115200);
  delay(100);

  Serial.println(F("Motor controller ready"));
  printHelp();
}

void loop() {
  serviceAxis(axes[0]);
  serviceAxis(axes[1]);
  readSerial();
}