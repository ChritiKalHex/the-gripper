/*
  Arduino UNO R3  ->  Oriental Motor UDK2120A driver  ->  PK266MA motor

  WIRING (direct, no transistors):
    D2  -> CW/PLS +     GND -> CW/PLS -
    D3  -> CCW/DIR +    GND -> CCW/DIR -
    D4  -> C.OFF +      GND -> C.OFF -     (optional)
    D5  -> O.HEAT +     GND -> COM         (optional)

  DRIVER SWITCHES:  ON | ON | F (400/rev) or H (800/rev) | 1P | N.O.

  SERIAL MONITOR: 115200 baud, Newline
    r 1      one revolution CW      r -0.5   half rev CCW
    m 400    move 400 steps         v 1500   max speed (steps/s)
    a 3000   accel (steps/s^2)      f / e    free / enable shaft
    p        print position         z        zero position
*/

const uint8_t PIN_PLS   = 2;
const uint8_t PIN_DIR   = 3;
const uint8_t PIN_COFF  = 4;
const uint8_t PIN_OHEAT = 5;

const long  STEPS_PER_REV = 400;    // 400 if switch on F, 800 if on H
float maxSpeed = 8000.0;            // steps/s
float accel    = 16000.0;            // steps/s^2
const float START_SPEED = 200.0;    // steps/s
const float SPEED_LIMIT = 14000.0;   // R3 practical ceiling

const unsigned int PULSE_ON_US  = 10;  // manual: >= 5 us
const unsigned int DIR_SETUP_US = 50;  // manual: >= 20 us

long position = 0;
bool dirCW    = false;
bool enabled  = true;

void pulse() {
  digitalWrite(PIN_PLS, HIGH);      // optocoupler ON
  delayMicroseconds(PULSE_ON_US);
  digitalWrite(PIN_PLS, LOW);       // ON->OFF = one step; idle OFF
}

void setDirection(bool cw) {
  if (cw == dirCW) return;
  digitalWrite(PIN_DIR, cw ? HIGH : LOW);
  dirCW = cw;
  delayMicroseconds(DIR_SETUP_US);
}

void enableMotor(bool on) {
  digitalWrite(PIN_COFF, on ? LOW : HIGH);
  enabled = on;
  if (on) delay(100);               // manual: wait ~100 ms before pulsing
  Serial.println(on ? F("Motor enabled") : F("Motor free"));
}

bool overheated() {
  return digitalRead(PIN_OHEAT) == LOW;   // N.O.: pulled LOW on overheat
}

bool moveSteps(long steps) {
  if (steps == 0) return true;
  if (!enabled)     { Serial.println(F("Motor free - send e first")); return false; }
  if (overheated()) { Serial.println(F("OVERHEAT - not moving"));     return false; }

  setDirection(steps > 0);
  long n = labs(steps);

  float vmax = constrain(maxSpeed, START_SPEED, SPEED_LIMIT);
  long ramp = (long)((vmax * vmax - START_SPEED * START_SPEED) / (2.0 * accel));
  if (ramp > n / 2) ramp = n / 2;

  unsigned long t = micros();
  long done = 0;
  for (long i = 0; i < n; i++) {
    long fromEnd = (i < n - 1 - i) ? i : (n - 1 - i);
    float v = (fromEnd < ramp)
                ? sqrt(START_SPEED * START_SPEED + 2.0 * accel * fromEnd)
                : vmax;
    if (v > vmax) v = vmax;

    pulse();
    done++;

    t += (unsigned long)(1000000.0 / v);
    while ((long)(micros() - t) < 0) { }

    if ((i & 0x3F) == 0 && overheated()) {
      Serial.println(F("OVERHEAT - aborted"));
      break;
    }
  }

  position += (steps > 0) ? done : -done;
  Serial.print(F("Done. Position = "));
  Serial.println(position);
  return done == n;
}

void printHelp() {
  Serial.println(F("r <revs> | m <steps> | v <steps/s> | a <steps/s^2> | f | e | p | z"));
  Serial.print(F("Steps/rev=")); Serial.print(STEPS_PER_REV);
  Serial.print(F(" maxSpeed="));  Serial.print(maxSpeed);
  Serial.print(F(" accel="));     Serial.println(accel);
}

void setup() {
  digitalWrite(PIN_PLS, LOW);  pinMode(PIN_PLS, OUTPUT);
  digitalWrite(PIN_DIR, LOW);  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_COFF, LOW); pinMode(PIN_COFF, OUTPUT);
  pinMode(PIN_OHEAT, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.setTimeout(100);
  delay(100);
  Serial.println(F("UDK2120A / PK266MA ready"));
  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char  cmd = tolower(line.charAt(0));
  float val = line.substring(1).toFloat();

  switch (cmd) {
    case 'r': moveSteps((long)round(val * STEPS_PER_REV)); break;
    case 'm': moveSteps((long)val); break;
    case 'v': maxSpeed = constrain(val, START_SPEED, SPEED_LIMIT);
              Serial.print(F("maxSpeed=")); Serial.println(maxSpeed); break;
    case 'a': accel = (val < 1) ? 1 : val;
              Serial.print(F("accel=")); Serial.println(accel); break;
    case 'f': enableMotor(false); break;
    case 'e': enableMotor(true);  break;
    case 'p': Serial.print(F("Position=")); Serial.println(position); break;
    case 'z': position = 0; Serial.println(F("Zeroed")); break;
    default:  printHelp(); break;
  }
}