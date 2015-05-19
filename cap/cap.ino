#include <Adafruit_NeoPixel.h>
#include <dht11.h>
#include <Servo.h> 
dht11 DHT11;

#define DHT11PIN 8
Servo leftWing, rightWing;
int pos = 0;

const int ledPin = 13;      // led connected to digital pin 13
const int temperatureSensor = A2; // the piezo is connected to analog pin 0
const int vibrationSensor = A5;
const int airSensor = A2;
const int UVOUT = A0; //Output from the sensor
const int REF_3V3 = A1; //3.3V power on the Arduino board


const int threshold = 100;  // threshold value to decide when the detected sound is a knock or not


// these variables will change:
int tempersature_read = 0;      // variable to store the value read from the sensor pin
int vibration_read = 0;
int air_read = 0;
int uv_read = 0;
int ledState = LOW;         // variable used to store the last LED status, to toggle the light

int debugEnable = 1;

Adafruit_NeoPixel ledA = Adafruit_NeoPixel(5, 1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledB = Adafruit_NeoPixel(5,2,NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledC = Adafruit_NeoPixel(5, 3, NEO_GRB + NEO_KHZ800);

uint16_t ledA_level=0;  




void setup(){
  
  leftWing.attach(9);  // attaches the servo on pin 9 to the servo object 
  rightWing.attach(10);  // attaches the servo on pin 10 to the servo object
  
  rightWing.write(0);
  leftWing.write(0);
  

 pinMode(ledPin, OUTPUT); // declare the ledPin as as OUTPUT
 pinMode(1,OUTPUT);
 pinMode(2,OUTPUT);
 pinMode(3,OUTPUT);
 Serial.begin(115200);       // use the serial port
}

void setLedA(int ledID, uint16_t level) {
  if(ledID==1) {
  for(uint16_t i=0; i<ledA.numPixels(); i++) {
    if(i<=level) {
      Serial.println(i);
      ledA.setPixelColor(i, ledA.Color(155, 89, 182));
    } else {
      ledA.setPixelColor(i, ledA.Color(0,0,0));
    }
    
  }
  ledA.show();
  delay(10);
  } else if(ledID==2) {
  for(uint16_t i=0; i<ledB.numPixels(); i++) {
    if(i<=level) {
      ledB.setPixelColor(i, ledB.Color(211, 84, 0));
    } else {
      ledB.setPixelColor(i, ledB.Color(0,0,0));
    }
    
  }
  ledA.show();
  delay(10);
  } else if(ledID==1) {
  for(uint16_t i=0; i<ledC.numPixels(); i++) {
    if(i<=level) {
      ledC.setPixelColor(i, ledC.Color(52, 152, 219));
    } else {
      ledC.setPixelColor(i, ledC.Color(0,0,0));
    }
    
  }
  ledA.show();
  delay(10);
  }
  
 
}
 
int readDHT11() {
   int chk = DHT11.read(DHT11PIN);

  Serial.print("Read sensor: ");
  int dataOK=0;
  switch (chk)
  {
    case DHTLIB_OK: 
		Serial.println("OK"); 
		dataOK = 1;
                break;
    case DHTLIB_ERROR_CHECKSUM: 
		Serial.println("Checksum error"); 
		break;
    case DHTLIB_ERROR_TIMEOUT: 
		Serial.println("Time out error"); 
		break;
    default: 
		Serial.println("Unknown error"); 
		break;
  }
  if(dataOK) {
  Serial.print("Humidity (%): ");
  Serial.println((float)DHT11.humidity, 2);

  Serial.print("Temperature (°C): ");
  Serial.println((float)DHT11.temperature, 2);
  }
  return dataOK;
  

}

void loop() {
  // read the sensor and store it in the variable sensorReading:
  tempersature_read = analogRead(temperatureSensor);    
  vibration_read = analogRead(vibrationSensor);    
  air_read = analogRead(airSensor);

  int DHTReady=0;
  
   int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 1, 4); //Convert the voltage to a UV intensity level
  int uvlevel = uvIntensity;
  int temperature = -1;
  int humidity = -1;
  if(debugEnable ) {
    DHTReady = readDHT11();
    temperature =  map(DHT11.temperature,20,30,1,4);
    humidity = map(DHT11.humidity, 20,80,1,4);
    
     Serial.print("[Sensor] ");
    Serial.print("UV Intensity : ");
    Serial.print(uvlevel);
    Serial.print(" ||| Temp : ");
    Serial.print(temperature);
    Serial.print("   ");
    Serial.print(" ||| Humidity : ");
    Serial.print(humidity);
    Serial.println("");
    
    
    setLedA(1, uvlevel); 
    setLedA(2, temperature);
    setLedA(3, humidity);
    
    if(uvlevel > 2.5)
    {
      leftWing.write(90);
      rightWing.write(90);
    }
    else if(uvlevel < 2.5)
    {
      leftWing.write(0);
      rightWing.write(0);
    }   
  }
     
    delay(100);
}



int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

