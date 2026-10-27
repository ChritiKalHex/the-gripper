#include <Arduino.h>
#include <Stepper.h>

const int stepsPerRevolution = 1024;

// Stepper One (Base --> Boom 1)
// in1, in2, in3, in4 --> a, b, c, d
int di_a = 8;
int di_b = 9;
int di_c = 10;
int di_d = 11;

Stepper stepperOne(stepsPerRevolution, di_a, di_b, di_c, di_d);

// Stepper Two (Boom 1 --> Boom 2)
// in1, in2, in3, in4 --> e, f, g, h
int di_e = 8;
int di_f = 9;
int di_g = 10;
int di_h = 11;

Stepper stepperTwo(stepsPerRevolution, di_e, di_f, di_g, di_h);

void setup() {
  stepperOne.setSpeed(10);
  stepperTwo.setSpeed(10);
  Serial.begin(9600);
}

void loop() {
  stepperOne.step(stepsPerRevolution);
  stepperTwo.step(stepsPerRevolution);
  delay(1000); // delay to not blow it up
}