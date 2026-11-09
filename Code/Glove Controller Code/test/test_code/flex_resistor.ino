const int flexPin = A3; // Pin connected to voltage divider output

void setup()
{
    Serial.begin(4800);
    pinMode(flexPin, INPUT);
}

void loop()
{
    // Read the ADC, and calculate voltage and resistance from it
    int flexSensor = analogRead(flexPin);

    Serial.println(flexSensor);

    delay(1000);
}