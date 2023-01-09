#include <Arduino.h>
#include <DHT.h>
#include <Servo.h>

#define DHTTYPE DHT11
#define pinServo D4
#define pinDHT D1
#define pinGas D3
#define pinLEDout D7 // 13
#define pinLEDin D6  // 12
#define pinLUMEN D5  // 4

struct
{
  float temperatureValue = 0.0;
  float humidityValue = 0.0;
  bool gasValue = false;
  bool isNIGHT = false;
  bool lightAutomation = true;
  bool ledOutVal = false;
  bool ledInVal = false;
  bool isLocked = true;
} currentState, previousState;

Servo myservo;
DHT dht(pinDHT, DHTTYPE);

void openDoor()
{
  myservo.write(180);
  Serial.println("Open door.");
  currentState.isLocked = false;
}

void closeDoor()
{
  myservo.write(0);
  Serial.println("Clsoe door.");
  currentState.isLocked = true;
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
  float newT = dht.readTemperature();
  if (isnan(newT))
  {
    Serial.println("Failed to read from DHT sensor!");
    return -1;
  }
  else
  {
    return newT;
  }
}

float readHumiditySensor()
{
  float newH = dht.readHumidity();
  if (isnan(newH))
  {
    Serial.println("Failed to read from DHT sensor!");
    return -1;
  }
  else
  {
    return newH;
  }
  return -1;
}

void turnLEDinON()
{
  digitalWrite(pinLEDin, HIGH);
  currentState.ledInVal = true;
}

void turnLEDinOFF()
{
  digitalWrite(pinLEDin, LOW);
  currentState.ledInVal = false;
}

void turnLEDoutON()
{
  digitalWrite(pinLEDout, HIGH);
  currentState.ledOutVal = true;
}

void turnLEDoutOFF()
{
  digitalWrite(pinLEDout, LOW);
  currentState.ledOutVal = false;
}

void LEDup()
{
  if (currentState.lightAutomation) // Light Automation ON
  {
    if (currentState.isNIGHT)
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
  Serial.begin(115200);
  pinMode(pinLUMEN, INPUT);   // LUMEN pin as input.
  pinMode(pinLEDin, OUTPUT);  // LED pin as output.
  pinMode(pinLEDout, OUTPUT); // LED pin as output.
  pinMode(pinGas, INPUT);
  myservo.attach(pinServo);
  dht.begin();
}

void readSensors()
{
  currentState.isNIGHT = digitalRead(pinLUMEN);
  currentState.temperatureValue = readTemperatureSensor();
  currentState.humidityValue = readHumiditySensor();
  currentState.gasValue = readGasSensor();
  // currentState.lightAutomation = true;
  // currentState.ledOutVal = false;
  // currentState.ledInVal = false;
  // currentState.isLocked = true;
}

void algorithms()
{
  LEDup();
}

void loop()
{
  readSensors();
  algorithms();

  Serial.printf("T=%f | H=%f | Gas=%d | Night=%d \n", currentState.temperatureValue, currentState.humidityValue, currentState.gasValue, currentState.isNIGHT);

  // Serial.print(currentState.temperatureValue);
  // Serial.print("    ");
  // Serial.print(currentState.humidityValue);
  // Serial.print("    ");
  // Serial.print(currentState.gasValue);
  // Serial.print("    ");
  // Serial.print(currentState.ledInVal);
  // Serial.print("    ");
  // Serial.print(currentState.ledOutVal);
  // Serial.print("    ");
  // Serial.print(currentState.lightAutomation);
  // Serial.print("    ");
  // Serial.println();
}