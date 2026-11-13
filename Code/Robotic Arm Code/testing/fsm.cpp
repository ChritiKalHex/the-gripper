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

enum class State { START, GRAB, INSERT, SPIN, TAKEOUT, END };

State currentState = State::START;

const __FlashStringHelper *stateName(State s) {
  switch (s) {
    case State::START:   return F("START");
    case State::GRAB:    return F("GRAB");
    case State::INSERT:  return F("INSERT");
    case State::SPIN:    return F("SPIN");
    case State::TAKEOUT: return F("TAKEOUT");
    case State::END:     return F("END");
  }
  return F("?");
}

void setState(State newState) {
  currentState = newState;
  Serial.print(F("State: "));
  Serial.println(stateName(newState));

  //random number for where to move, hopefully replace this with points with the kinematics stuff
  switch (newState) {
    case State::START:
      break;
    case State::GRAB:
      startMove(axes[0], 400);
      break;
    case State::INSERT:
      startMove(axes[1], 800);
      break;
    case State::SPIN:
      startMove(axes[0], 1600);
      break;
    case State::TAKEOUT:
      startMove(axes[1], -800);
      break;
    case State::END:
      stopAxis(axes[0]);
      stopAxis(axes[1]);
      break;
  }
}

void nextState() {
  if (currentState == State::END) {
    Serial.println(F("done :)"));
    return;
  } else {
    setState(static_cast<State>(static_cast<int>(currentState) + 1));
  }
}

//press space to go to next state 
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c != ' ') continue;

    if (axes[0].phase == IDLE && axes[1].phase == IDLE) {
      nextState();
    } else {
      Serial.println(F("not ready"));
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

  Serial.print(F("State: "));
  Serial.println(stateName(currentState));
}

void loop() {
  serviceAxis(axes[0]);
  serviceAxis(axes[1]);
  readSerial();
}