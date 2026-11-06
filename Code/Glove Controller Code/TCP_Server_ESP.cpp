// Test out esp32 connection with laptop as arduino board

#include <WiFi.h>

const char *ssid = "GloveNetwork";
const char *pass = "grippy";
const char *host = "192.168.1.100"; // laptop now, Uno later
const uint16_t port = 8080;
WiFiClient client;

void setup()
{
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
    delay(500);
}

void loop()
{
  if (!client.connected())
  {
    client.connect(host, port);
    delay(500);
    return;
  }
  float temp = 20.0 + random(0, 100) / 10.0; // fake reading
  client.printf("temp=%.1f\n", temp);
  delay(1000);
}