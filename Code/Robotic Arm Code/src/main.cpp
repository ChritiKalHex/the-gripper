#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

// TWO(?) UDK2120A Drivers + TWO PK266MA Stepper Motors
// Arduino UNO R3

// Motor 1: Base -> Boom 1
const uint8_t M1_PLS   = 2;
const uint8_t M1_DIR   = 3;
const uint8_t M1_COFF  = 4;
const uint8_t M1_OHEAT = 5;

// Motor 2: Boom 1 -> Boom 2
const uint8_t M2_PLS   = 6;
const uint8_t M2_DIR   = 7;
const uint8_t M2_COFF  = 8;
const uint8_t M2_OHEAT = 9;

// Set to 400 for driver switch F
// or 800 for H??? how does this work
const long STEPS_PER_REV = 400;

float maxSpeed = 8000.0;
float accel = 16000.0;

const float START_SPEED = 200.0;
const float SPEED_LIMIT = 14000.0;

const unsigned int PULSE_ON_US = 10;
const unsigned int DIR_SETUP_US = 50;

// Motor Data

struct Motor {
  uint8_t pls;
  uint8_t dir;
  uint8_t coff;
  uint8_t oheat;

  long position;
  bool dirCW;
  bool enabled;
};

Motor motor1 = {
  M1_PLS, M1_DIR, M1_COFF, M1_OHEAT,
  0, false, true
};

Motor motor2 = {
  M2_PLS, M2_DIR, M2_COFF, M2_OHEAT,
  0, false, true
};

// Low-Level Control Things

void pulse(Motor &m) {
  digitalWrite(m.pls, HIGH);
  delayMicroseconds(PULSE_ON_US);
  digitalWrite(m.pls, LOW);
}

void setDirection(Motor &m, bool cw) {
  if (cw == m.dirCW) return;

  digitalWrite(m.dir, cw ? HIGH : LOW);
  m.dirCW = cw;

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

// Movement
// Positive steps = CW; negative steps = CCW

bool moveSteps(Motor &m, long steps) {
  if (steps == 0) return true;

  if (!m.enabled) {
    Serial.println(F("Motor free"));
    return false;
  }

  if (overheated(m)) {
    Serial.println(F("OVERHEAT - do not move"));
    return false;
  }

  setDirection(m, steps > 0);

  long n = labs(steps);

  float vmax = constrain(
    maxSpeed, START_SPEED, SPEED_LIMIT
  );

  float safeAccel = (accel < 1.0) ? 1.0 : accel;

  long ramp = (long)(
    (vmax * vmax - START_SPEED * START_SPEED)
    / (2.0 * safeAccel)
  );

  if (ramp > n / 2) ramp = n / 2;

  unsigned long t = micros();
  long done = 0;

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

    pulse(m);
    done++;

    t += (unsigned long)(1000000.0 / v);

    while ((long)(micros() - t) < 0) {
      // Wait for the next step interval.
    }

    if ((i & 0x3F) == 0 && overheated(m)) {
      Serial.println(F("OVERHEAT - aborted"));
      break;
    }
  }

  if (steps > 0) {
    m.position += done;
  } else {
    m.position -= done;
  }

  Serial.print(F("Position = "));
  Serial.println(m.position);

  return done == n;
}

// Serial Command Help Print

void printHelp() {
  Serial.println();
  Serial.println(F("=== TWO MOTOR CONTROLLER ==="));
  Serial.println(F("m1 <steps>    Move motor 1"));
  Serial.println(F("m2 <steps>    Move motor 2"));
  Serial.println(F("r1 <revs>     Rotate motor 1"));
  Serial.println(F("r2 <revs>     Rotate motor 2"));
  Serial.println(F("b <s1> <s2>   Move both, sequentially"));
  Serial.println(F("v <steps/s>   Set maximum speed"));
  Serial.println(F("a <steps/s2>  Set acceleration"));
  Serial.println(F("e             Enable both motors"));
  Serial.println(F("f             Free both motors"));
  Serial.println(F("p             Print both positions"));
  Serial.println(F("z             Zero both positions"));
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

  Serial.begin(115200);
  Serial.setTimeout(100);

  delay(100);

  Serial.println(F("Two-motor controller ready"));
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
    // Sequential movement for now.
    // I don't want to do synchronized movement.
    moveSteps(motor1, (long)x);
    moveSteps(motor2, (long)y);
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
    Serial.print(F("Motor 1 position: "));
    Serial.println(motor1.position);

    Serial.print(F("Motor 2 position: "));
    Serial.println(motor2.position);
  }
  else if (cmd == "z") {
    motor1.position = 0;
    motor2.position = 0;
    Serial.println(F("Both positions zeroed"));
  }
  else {
    printHelp();
  }
}