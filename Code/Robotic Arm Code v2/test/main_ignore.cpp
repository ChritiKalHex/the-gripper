// main.cpp - ESP32-S3 + Bluepad32: Xbox controller triggers -> serial commands for the two-motor Uno sketch
// Same logic as gamepad_motors.c; only the input/serial plumbing changed.
#include <Arduino.h>
#include <Bluepad32.h>

// ---- Input mapping (edit these to remap) ----
// Bluepad32 triggers report 0..1023 (SDL reported 0..32767), so thresholds are scaled down.
const int  TRIGGER_ON  = 250;   // ~8000/32767 of full range
const int  TRIGGER_OFF = 125;   // ~4000/32767 of full range
const long CHUNK_STEPS = 4000;  // steps requested per held-trigger command

// ---- Serial link to the Uno ----
// Avoid GPIO35/36/37 on N16R8 boards (reserved for octal PSRAM).
const int UNO_RX_PIN = 18;      // ESP32 receives here  <- Uno TX (through a voltage divider!)
const int UNO_TX_PIN = 17;      // ESP32 sends here     -> Uno RX

// ---- State (was the App struct) ----
ControllerPtr pad = nullptr;
unsigned long readyAt = 0;
bool moving[2] = { false, false };
bool wanted[2] = { false, false };
char lineBuf[64];
int  lineLen = 0;

void sendCmd(const char *s) {
  Serial1.print(s);
  Serial.print("-> ");
  Serial.print(s);
}

void stopAll() {
  sendCmd("x\n");
  moving[0] = moving[1] = false;
}

void handleReply(const char *line) {
  Serial.print("<- ");
  Serial.println(line);
  int n;
  if (sscanf(line, "Motor %d done", &n) == 1 && n >= 1 && n <= 2) {
    moving[n - 1] = false;
  } else if (strstr(line, "OVERHEAT")) {
    moving[0] = moving[1] = false;
  }
}

void pumpSerial() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        lineLen = 0;
        handleReply(lineBuf);
      }
    } else if (lineLen < (int)sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ---- Bluepad32 callbacks (replace SDL_EVENT_GAMEPAD_ADDED / REMOVED) ----
void onConnectedController(ControllerPtr ctl) {
  if (pad == nullptr) {
    pad = ctl;
    Serial.println("Controller connected");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (pad == ctl) {
    pad = nullptr;
    stopAll();                       // fail safe: controller gone = motors stop
    Serial.println("Controller disconnected");
  }
}

void setup() {
  Serial.begin(115200);                                        // USB debug log
  Serial1.begin(115200, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);   // link to the Uno

  BP32.setup(&onConnectedController, &onDisconnectedController);
  // BP32.forgetBluetoothKeys();   // uncomment once if pairing gets stuck, then re-comment

  readyAt = millis() + 2000;       // give the Uno time to boot
}

void loop() {
  BP32.update();                   // must run often; drives Bluetooth + callbacks
  pumpSerial();

  if (millis() >= readyAt && pad && pad->isConnected()) {
    // 1) read triggers (with hysteresis). brake() = left trigger, throttle() = right trigger
    int  value[2] = { pad->brake(), pad->throttle() };
    bool rev[2]   = { pad->l1(), pad->r1() };

    for (int i = 0; i < 2; i++) {
      if (value[i] > TRIGGER_ON)       wanted[i] = true;
      else if (value[i] < TRIGGER_OFF) wanted[i] = false;
    }

    // 2) a released trigger -> stop (firmware's 'x' stops both axes)
    bool needStop = false;
    for (int i = 0; i < 2; i++)
      if (!wanted[i] && moving[i]) needStop = true;
    if (needStop) stopAll();

    // 3) start (or restart) any held trigger that isn't running
    for (int i = 0; i < 2; i++) {
      if (wanted[i] && !moving[i]) {
        char cmd[32];
        snprintf(cmd, sizeof cmd, "m%d %ld\n", i + 1, rev[i] ? -CHUNK_STEPS : CHUNK_STEPS);
        sendCmd(cmd);
        moving[i] = true;
      }
    }
  }

  delay(5);
}