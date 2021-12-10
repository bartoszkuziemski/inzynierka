#include <Servo.h>
#include <SPI.h>

#define ThermistorPin A0 // thermistor analog input
#define HeaterAnalogPin A1 // heater analog input from Trend 0-10V
#define FanAnalogPin A2 // fan analog input from Trend 0-10V
#define GoodCondPin A3 // good conditions analog input
#define HeaterPWM_Pin 5
#define FanPWM_Pin 6
#define ServoPin_PWM 3
#define DAC_CS_PIN 10 // Chip select pin for DAC

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

// DAC control
int DAC_Val = 0;

// Servo control
int GoodCondReadVal = 0; // analog value of good conditions (0-1023)
Servo myServo;
int ServoPos = 0; // position of servo
bool ServoIsUp = false;
bool ServoIsDown = false;

void setup() {
  // Servo control
  Serial.begin(9600);
  myServo.attach(ServoPin_PWM);
  pinMode(ServoPin_PWM, OUTPUT);

  // DAC control
  pinMode(DAC_CS_PIN, OUTPUT);
  digitalWrite(DAC_CS_PIN, HIGH);
  SPI.begin();

  // Heater control
  pinMode(HeaterPWM_Pin, OUTPUT);

  // Fan control
  pinMode(FanPWM_Pin, OUTPUT);
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

  //      Serial port print to check if values are correct
  //  Serial.print(R2);
  //  Serial.print("      ");
  //  Serial.print(ThermistorReadVal);
  //  Serial.print("      ");
  //  Serial.print(T2 - 273.15);
  //  Serial.println(" °C");
}

void set_DAC_Voltage(int value)
{
  byte DAC_Register = 0b00110000;
  int temp_2byte_value = 0b0000000011111111;
  byte FirstByte = (value >> 8) | DAC_Register;
  byte SecondByte = value & temp_2byte_value;

  noInterrupts();
  digitalWrite(DAC_CS_PIN, LOW);
  SPI.transfer(FirstByte);
  SPI.transfer(SecondByte);
  digitalWrite(DAC_CS_PIN, HIGH);
  interrupts();
}

float f_map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ServoControl()
{
  if (GoodCondReadVal > 512)
  {
    ServoIsDown = false;
    if (!ServoIsUp)
    {
      for (ServoPos = 0; ServoPos < 90; ServoPos += 1)
      {
        myServo.write(ServoPos);
        delay(50);
      }
      ServoIsUp = true;
    }
  }
  else
  {
    ServoIsUp = false;
    if (!ServoIsDown)
    {
      for (ServoPos = 90; ServoPos >= 0; ServoPos -= 1)
      {
        myServo.write(ServoPos);
        delay(50);
      }
      ServoIsDown = true;
    }
  }
}

void AnalogToPWM(int analogPin, int PwmPin)
{
  int ReadVal = analogRead(analogPin);
  int PwmVal = map(ReadVal, 0 , 1023, 0 , 255);
  analogWrite(PwmPin, PwmVal);
}

void loop() {
  TemperatureRead();
  
  DAC_Val = f_map(T2, 273, 373, 0, 4047); // map temperature from 273-373 (0-100 Celsius deg) to 0-4047
  set_DAC_Voltage(DAC_Val);
  // Serial.println(DAC_Val);

  AnalogToPWM(HeaterAnalogPin, HeaterPWM_Pin);
  AnalogToPWM(FanAnalogPin, FanPWM_Pin);

  GoodCondReadVal = analogRead(GoodCondPin);
  // Serial.println(GoodCondReadVal);

  ServoControl();

}
