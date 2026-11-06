#include <WiFiS3.h>

const char* ssid = "YourNetwork";
const char* pass = "YourPassword";
IPAddress serverIP(192, 168, 1, 50);  // the ESP32's IP from above
WiFiClient client;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void loop() {
  if (!client.connected()) client.connect(serverIP, 8080);

  if (client.connected()) {
    client.println("hello from uno");
    delay(100);
    while (client.available()) {
      Serial.println(client.readStringUntil('\n'));
    }
  }
  delay(1000);
}