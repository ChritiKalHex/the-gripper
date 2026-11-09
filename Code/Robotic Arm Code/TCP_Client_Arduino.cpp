#include <WiFiS3.h>
#include <WiFiUdp.h>

const char *WIFI_SSID = "123";
const char *WIFI_PASS = "123";
const uint16_t LISTEN_PORT = 4210;

WiFiUDP udp;
char buf[96];

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  }

  Serial.print("Connecting to WiFi");
  while (WiFi.begin(WIFI_SSID, WIFI_PASS) != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
  Serial.print("R4 IP address (put this in the sender): ");
  Serial.println(WiFi.localIP());

  udp.begin(LISTEN_PORT);
  Serial.println("Listening...");
}

void loop()
{
  if (udp.parsePacket())
  {
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len <= 0)
      return;
    buf[len] = '\0';

    unsigned long ts;
    int x1, y1, z1, x2, y2, z2;
    if (sscanf(buf, "%lu,%d,%d,%d,%d,%d,%d", &ts, &x1, &y1, &z1, &x2, &y2, &z2) == 7)
    {
      Serial.print("t=");
      Serial.print(ts);
      Serial.print("  A: ");
      Serial.print(x1);
      Serial.print(", ");
      Serial.print(y1);
      Serial.print(", ");
      Serial.print(z1);
      Serial.print("  B: ");
      Serial.print(x2);
      Serial.print(", ");
      Serial.print(y2);
      Serial.print(", ");
      Serial.println(z2);
    }
  }
}