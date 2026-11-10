#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

/*********** NETWORK SETTINGS ***********/
const char *WIFI_SSID = "123";
const char *WIFI_PASS = "123";
const char *RECEIVER_IP = "192.168.4.1"; // IP the R4 prints on boot (or laptop IP for testing)
const uint16_t RECEIVER_PORT = 4210;

WiFiUDP udp;

/*********** ADXL345 REGISTERS ***********/
#define ADXL_ADDR_A 0x53 // SDO -> GND
#define ADXL_ADDR_B 0x1D // SDO -> 3.3V

#define REG_DEVID 0x00
#define REG_BW_RATE 0x2C
#define REG_POWER_CTL 0x2D
#define REG_DATA_FORMAT 0x31
#define REG_DATAX0 0x32

/*********** CALIBRATION (per sensor) ***********/
// offsetAxis = 0.5 * (Accel+1g + Accel-1g)
// gainAxis   = 0.5 * ((Accel+1g - Accel-1g) / 1g)
struct Calib
{
  float offX, offY, offZ;
  float gainX, gainY, gainZ;
};

Calib calA = {0, 0, 0, 1, 1, 1};
Calib calB = {0, 0, 0, 1, 1, 1};

const unsigned long SEND_INTERVAL_MS = 20; // 50 Hz
unsigned long lastSend = 0;

/*********** ADXL345 HELPERS ***********/
void adxlWrite(uint8_t addr, uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool adxlInit(uint8_t addr)
{
  Wire.beginTransmission(addr);
  Wire.write(REG_DEVID);
  if (Wire.endTransmission() != 0)
    return false;
  Wire.requestFrom(addr, (uint8_t)1);
  if (!Wire.available() || Wire.read() != 0xE5)
    return false; // ADXL345 device ID

  adxlWrite(addr, REG_DATA_FORMAT, 0x08); // full resolution, +/-2g
  adxlWrite(addr, REG_BW_RATE, 0x0A);     // 100 Hz output data rate
  adxlWrite(addr, REG_POWER_CTL, 0x08);   // measurement mode
  return true;
}

bool adxlRead(uint8_t addr, int16_t &x, int16_t &y, int16_t &z)
{
  Wire.beginTransmission(addr);
  Wire.write(REG_DATAX0);
  if (Wire.endTransmission(false) != 0)
    return false;
  if (Wire.requestFrom(addr, (uint8_t)6) != 6)
    return false;

  uint8_t b[6];
  for (int i = 0; i < 6; i++)
    b[i] = Wire.read();
  x = (int16_t)((b[1] << 8) | b[0]);
  y = (int16_t)((b[3] << 8) | b[2]);
  z = (int16_t)((b[5] << 8) | b[4]);
  return true;
}

int applyCal(int16_t raw, float offset, float gain)
{
  return (int)lroundf((raw - offset) / gain);
}

/*********** WIFI ***********/
void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

/*********** MAIN ***********/
void setup()
{
  Serial.begin(115200);
  delay(1000);

  Wire.begin(); // default Metro ESP32-S2 SDA/SCL pins
  Wire.setClock(400000);

  Serial.println(adxlInit(ADXL_ADDR_A) ? "Sensor A (0x53) OK" : "Sensor A (0x53) NOT FOUND");
  Serial.println(adxlInit(ADXL_ADDR_B) ? "Sensor B (0x1D) OK" : "Sensor B (0x1D) NOT FOUND");

  connectWiFi();
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
    connectWiFi();

  if (millis() - lastSend >= SEND_INTERVAL_MS)
  {
    lastSend = millis();

    int16_t ax, ay, az, bx, by, bz;
    bool okA = adxlRead(ADXL_ADDR_A, ax, ay, az);
    bool okB = adxlRead(ADXL_ADDR_B, bx, by, bz);
    if (!okA || !okB)
    {
      Serial.println("I2C read failed, skipping sample");
      return;
    }

    // Packet format: timestamp_ms,x1,y1,z1,x2,y2,z2
    char packet[96];
    int len = snprintf(packet, sizeof(packet), "%lu,%d,%d,%d,%d,%d,%d",
                       millis(),
                       applyCal(ax, calA.offX, calA.gainX),
                       applyCal(ay, calA.offY, calA.gainY),
                       applyCal(az, calA.offZ, calA.gainZ),
                       applyCal(bx, calB.offX, calB.gainX),
                       applyCal(by, calB.offY, calB.gainY),
                       applyCal(bz, calB.offZ, calB.gainZ));

    udp.beginPacket(RECEIVER_IP, RECEIVER_PORT);
    udp.write((const uint8_t *)packet, len);
    udp.endPacket();

    Serial.println(packet);
  }
}