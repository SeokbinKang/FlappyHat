# Flappy Hat

- Flappy Hat website: http://cmsc838f-s15.wikispaces.com/FlappyHat
- Youtube video: 

_Description: An Intelligent Hat that could Automatically Make a Shade for you and Visualize things that could not be seen by the Naket Eye_

This wearable device is consisted of an Arduino that is connencted to two Sensors (UV Light Sensor & Temprature/Humidity Sensor) and two Servo Motors. This github repository is dedicated to the Arduino code that performs the following six tasks:

1. Initialize Components
2. Reads UV Light Sensor using ADC analogRead(int pinNumber) function
3. Reads Humidity/Temprature Sensor data using the readDHT11() function
4. Maps the raw output of the three Sensors into 5 discrete levels.
5. Sets the three LED strip values to the three Sensor Levels using setLedA(int LED_StripNumber, int value) function.
6. Finally if the UV Light Sensor value is higher than a threshold, then set the wings to 90 degrees using servoMotor.write(int degree) function.

## Project Setup:

_Materials Used_ 

1. Servo Motors x 2
2. Arduino Blend Micro
3. Humidity/Temprature Sensor (DHT11)
4. UV Light Sensor
5. Cap
6. Foam Core (For laser-cutting the wings)
7. 3D Printed shaft for holding the wings using Servo Motor Horn.
8. Wires, glue and other misc material
