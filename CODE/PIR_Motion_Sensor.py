import serial
import pyttsx3
import time

ser = serial.Serial('COM10', 9600) 
# text-to-speech engine
engine = pyttsx3.init()

while True:
    # Read data from serial port
    data = ser.readline().decode().strip()
    
    # Print received data for debugging
    print("Received data:", data)
    
    # Speak the received message
    engine.say(data)
    engine.runAndWait()
