#include <Arduino.h>

#define PIR_PIN PA1 

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Configure PIR pin as input
  pinMode(PIR_PIN, INPUT);
  
  // Configure LED pin as output
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Read PIR sensor output
  int pirState = digitalRead(PIR_PIN);
  
  if (pirState == HIGH) {
    Serial.println("Motion Detected");
    digitalWrite(LED_BUILTIN, HIGH); // Turn on the LED
  } else {
    Serial.println("No Motion Detected");
    digitalWrite(LED_BUILTIN, LOW); // Turn off the LED
  }
  
  delay(1000); // Adjust delay as needed
}
