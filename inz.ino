#include <Servo.h>

#define ThermistorPin A0 // thermistor analog input
#define GoodCondPin A1 // good conditions analog input
#define ServoControlPin 9 // PWM Pin for servo control

// Temperature read
float B_Value = 3950;
float R1 = 100000;
float T1 = 298.15;
float R2 ;
float T2 ;

float a ;
float b ;
float c ;
float d ;
float e = 2.718281828 ;
float ThermistorReadVal = 0;

// Servo control
int GoodCondReadVal=0; // analog value of good conditions (0-1023)
Servo myServo;
int ServoPos = 0; // position of servo
bool ServoIsUp=false; 
bool ServoIsDown=false;

void setup() {
  Serial.begin(9600);
  myServo.attach(ServoControlPin);
  pinMode(ServoControlPin, OUTPUT);
}

void TemperatureRead()
{
  ThermistorReadVal = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / ThermistorReadVal - 1.0);

  a = 1 / T1;
  b = log10(R1 / R2);
  c = b / log10(e);
  d = c / B_Value ;
  T2 = 1 / (a - d);

// Serial port print to check if values are correct
//  Serial.print(R2);
//  Serial.print("      ");
//  Serial.print(ThermistorReadVal);
//  Serial.print("      ");
//  Serial.print(T2 - 273.15);
//  Serial.println(" °C");
}

void loop() {

GoodCondReadVal=analogRead(GoodCondPin);
Serial.println(GoodCondReadVal);

TemperatureRead();

if(GoodCondReadVal>512)
  {
    ServoIsDown=false;
    if(!ServoIsUp)
    {
      for (ServoPos = 0; ServoPos < 90; ServoPos += 1)
      {
        myServo.write(ServoPos);
        delay(50);
      }
      ServoIsUp=true;
    }
  }
  else
  {
    ServoIsUp=false;
    if(!ServoIsDown)
    {
      for (ServoPos = 90; ServoPos >= 0; ServoPos -= 1)
      {
        myServo.write(ServoPos);
        delay(50);
      }
      ServoIsDown=true;
    }
  }


}
