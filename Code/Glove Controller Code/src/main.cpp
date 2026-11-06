// This file is part of the Arduino project. It contains the main code for the glove controller.
// It collects data from the sensors and sends it to the Arduino board for processing. Then,
// it sends the processed data to another microcontroller or computer for further analysis and control of the glove's movements.

#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup()
{
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop()
{
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}