/*
  Vehicle Theft Control System using Engine Temperature Monitoring
  Author: Kushal B D
  Components: Arduino Uno, LM35, Relay, GSM (simulated), GPS, Hidden Switch, Buzzer
  Date: 2025-09-18
*/

#include <SoftwareSerial.h>

// Pin Definitions
const int lm35Pin = A0;       // LM35 temperature sensor connected to A0
const int relayPin = 8;       // Relay module connected to digital pin 8
const int buzzerPin = 9;      // Buzzer for alarm
const int switchPin = 7;      // Hidden switch for authorization

// GSM + GPS simulation via Serial Monitor
SoftwareSerial gpsSerial(2, 3); // RX, TX (GPS module if connected)

void setup() {
  Serial.begin(9600);        // Serial monitor for GSM alerts
  gpsSerial.begin(9600);     // GPS serial communication
  pinMode(relayPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);
  digitalWrite(relayPin, HIGH); // Engine ON (relay closed by default)

  Serial.println("Vehicle Theft Control System Initialized...");
}

void loop() {
  int switchState = digitalRead(switchPin);  // Read hidden switch
  int tempReading = analogRead(lm35Pin);     // Read LM35 value
  float temperature = (tempReading * 5.0 * 100.0) / 1024.0; // Convert to Celsius

  // Print sensor values
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" C | Switch: ");
  Serial.println(switchState == HIGH ? "ON" : "OFF");

  // Theft detection logic
  if (switchState == LOW || temperature > 50.0) { // Hidden switch OFF or abnormal temp
    digitalWrite(relayPin, LOW);   // Cut off engine
    digitalWrite(buzzerPin, HIGH); // Sound alarm
    Serial.println("ALERT: Unauthorized access detected!");
    Serial.println("Sending SMS: Theft detected. Engine stopped.");
    
    // GPS location data (simulated, replace with actual GPS read)
    if (gpsSerial.available()) {
      String gpsData = gpsSerial.readStringUntil('\n');
      Serial.print("GPS Data: ");
      Serial.println(gpsData);
    } else {
      Serial.println("GPS Location: LAT 12.2958, LON 76.6394 (Simulated)");
    }
  } else {
    digitalWrite(relayPin, HIGH);  // Engine ON
    digitalWrite(buzzerPin, LOW);  // No alarm
    Serial.println("Engine running normally.");
  }

  delay(2000); // Delay for stability
}
