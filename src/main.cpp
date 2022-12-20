#include <Arduino.h>
#include <DHT.h>
#include <Servo.h>

#define DHTTYPE DHT11

#define pinServo 2
#define pinDHT 5
#define pinGas 0
#define pinLEDout D7 // 13
#define pinLEDin D6  // 12
#define pinLUMEN D5  // 4

bool lightAutomation = true;
bool ledOutVal = false;
bool ledInVal = false;

float temperatureValue = 0.0;
float humidityValue = 0.0;
int gasValue = 0;
int pos = 0;

Servo myservo;
DHT dht(pinDHT, DHTTYPE);

void openDoor()
{
  myservo.write(180);
  Serial.println("Open door.");
}

void closeDoor()
{
  myservo.write(0);
  Serial.println("Clsoe door.");
}

int readGasSensor()
{
  int g = digitalRead(pinGas);
  if (isnan(g))
  {
    Serial.println("Failed to read from MQ-5 sensor!");
    return -1;
  }
  else
    return g;
}
float readTemperatureSensor()
{
  float t = 0.0;

  float newT = dht.readTemperature();
  if (isnan(newT))
  {
    Serial.println("Failed to read from DHT sensor!");
    return -1;
  }
  else
  {
    t = newT;
    Serial.println(t);
    return t;
  }
}
float readHumiditySensor()
{
  float h = 0.0;

  float newH = dht.readHumidity();
  if (isnan(newH))
  {
    if (isnan(newH))
    {
      Serial.println("Failed to read from DHT sensor!");
      return -1;
    }
    else
    {
      h = newH;
      Serial.println(h);
      return h;
    }
  }
  return -1;
}
void turnLEDinON()
{
  digitalWrite(pinLEDin, HIGH);
}
void turnLEDinOFF()
{
  digitalWrite(pinLEDin, LOW);
}
void turnLEDoutON()
{
  digitalWrite(pinLEDout, HIGH);
}
void turnLEDoutOFF()
{
  digitalWrite(pinLEDout, LOW);
}
void LEDup()
{
  if (lightAutomation) // Light Automation ON
  {
    int isNIGHT = digitalRead(pinLUMEN);

    if (isNIGHT)
    {
      turnLEDoutON();
      turnLEDinON();
    }
    else
    {
      turnLEDoutOFF();
      turnLEDinOFF();
    }
  }
  else // Light Automation OFF
  {
    if (pinLEDout)
    {
      turnLEDoutON();
    }
    else
    {
      turnLEDoutOFF();
    }

    if (pinLEDin)
    {
      turnLEDinON();
    }
    else
    {
      turnLEDinOFF();
    }
  }
}

void setup()
{
  pinMode(pinLUMEN, INPUT);   // LUMEN pin as input.
  pinMode(pinLEDin, OUTPUT);  // LED pin as output.
  pinMode(pinLEDout, OUTPUT); // LED pin as output.
  pinMode(pinGas, INPUT);
  myservo.attach(pinServo);
  dht.begin();
}

void loop()
{
  Serial.print(temperatureValue);
  Serial.print("    ");
  Serial.print(humidityValue);
  Serial.print("    ");
  Serial.print(gasValue);
  Serial.print("    ");
  Serial.print(ledInVal);
  Serial.print("    ");
  Serial.print(ledOutVal);
  Serial.print("    ");
  Serial.print(lightAutomation);
  Serial.print("    ");
}