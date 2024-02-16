# PIR Motion Sensor with STM32

![Img5](https://github.com/AWESOME04/PIR-Motion-Sensor-Light-Control-with-STM32H743/assets/102630199/b11cdabe-1a28-4a48-90c1-d0a24fc69570)


This project demonstrates how to interface an HC-SR501 Pyroelectric PIR infrared motion sensor with an STM32 microcontroller to detect motion.

## Introduction

The HC-SR501 PIR motion sensor detects temperature changes caused by movement and sends signals to the STM32. The microcontroller interprets these signals to determine if motion has occurred, triggering actions such as activating lights or sending notifications.

## Hardware Components

- STM32H743 Nucleo-144
- HC-SR501 PIR Motion Sensor
- LED
- Jumper Wires

## Setup

1. Connect the HC-SR501 PIR motion sensor to the STM32 using the specified pins.
2. Upload the provided Arduino code to the STM32.
3. Monitor the sensor output using the Arduino Serial Monitor.

## Wiring

| STM32 Pin | PIR Motion Sensor Pin |
|-----------|-----------------------|
| 5V        | VCC                   |
| GND       | GND                   |
| D2        | OUT                   |

## Arduino Code

```cpp
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
```

## Serial Monitor Output

![image](https://github.com/AWESOME04/PIR-Motion-Sensor-Light-Control-with-STM32H743/assets/102630199/dda840b1-4c71-4f95-9553-ec1bf664e600)


## Python Script

```python
import serial
import pyttsx3
import time

ser = serial.Serial('COM10', 9600) 
engine = pyttsx3.init()

while True:
    data = ser.readline().decode().strip()
    print("Received data:", data)
    engine.say(data)
    engine.runAndWait()
```
## Python Script Output

![image](https://github.com/AWESOME04/PIR-Motion-Sensor-Light-Control-with-STM32H743/assets/102630199/ad7227aa-1f27-4efb-b123-2aac54290ea8)


## Schematics

![image](https://github.com/AWESOME04/PIR-Motion-Sensor-Light-Control-with-STM32H743/assets/102630199/a23f0158-5e93-41f7-82ed-b2a6f32a8ffd)


## Data Visualization

![image](https://github.com/AWESOME04/PIR-Motion-Sensor-Light-Control-with-STM32H743/assets/102630199/9224a909-5e75-4cf3-8666-6e2377339206)


## Applications

    - Home Automation
    - Security Systems
    - Robotics
    - Industrial Automation

## Contributors
    EVANS ACHEAMPONG – 10987644						
    EDWARD AYIREBI ACQUAH - 10986982
    AMOAH OFORI DARKWAH - 10949533 						
    BANI BERES ETORNAM – 10948391
    EYRAM AHETO - 10987509


## Conclusion

Interfacing the HC-SR501 Pyroelectric PIR infrared motion sensor with an STM32 provides a cost-effective solution for motion detection in various projects.


## REFERENCES

[1] https://arduinogetstarted.com/tutorials/arduino-motion-sensor

[2] https://circuitdigest.com/microcontroller-projects/interface-pir-sensor-with-arduino

[3] https://www.instructables.com/How-to-Use-a-PIR-Motion-Sensor-With-Arduino/

[4] https://www.circuits-diy.com/pir-motion-sensor-arduino-tutorial/

[5] https://www.phippselectronics.com/controlling-arduino-by-voice-with-python/

