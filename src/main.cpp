#include <Arduino.h>
#include <DHT.h>
#include <Servo.h>
#include <ServerHeader.h>

#define EVERY_MS(x)                  \
  static uint32_t tmr;               \
  bool flag = millis() - tmr >= (x); \
  if (flag)                          \
    tmr = millis();                  \
  if (flag)

#define DHTTYPE DHT11
#define pinServo D4
#define pinDHT D1
#define pinGas D6
#define pinLEDout D7 // 13
#define pinLEDin D3  // 12
#define pinLUMEN D5  // 4

struct
{
  float temperatureValue = 0.0;
  float humidityValue = 0.0;
  bool gasValue = false;
  bool isNIGHT = false;
  bool lightAutomation = true;
  bool ledOutVal = true;
  bool ledInVal = true;
  bool isLocked = true;
} currentState, previousState;
bool needToNotify = true;

Servo myservo;
DHT dht(pinDHT, DHTTYPE);

void openDoor()
{
  myservo.write(180);
  Serial.println("Opened door.");
  currentState.isLocked = false;
}

void closeDoor()
{
  myservo.write(0);
  Serial.println("Closed door.");
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
  Serial.println("Turned in led ON.");
}

void turnLEDinOFF()
{
  digitalWrite(pinLEDin, LOW);
  currentState.ledInVal = false;
  Serial.println("Turned in led OFF.");
}

void turnLEDoutON()
{
  digitalWrite(pinLEDout, HIGH);
  currentState.ledOutVal = true;
  Serial.println("Turned out led ON.");
}

void turnLEDoutOFF()
{
  digitalWrite(pinLEDout, LOW);
  currentState.ledOutVal = false;
  Serial.println("Turned out led OFF.");
}

void turnAutomationON()
{
  currentState.lightAutomation = true;
  Serial.println("Turned automation ON.");
}

void turnAutomationOFF()
{
  currentState.lightAutomation = false;
  Serial.println("Turned automation OFF.");
  turnLEDinOFF();
  turnLEDoutOFF();
}

void LEDup()
{
  if (currentState.lightAutomation) // Light Automation ON
  {
    if (currentState.isNIGHT != previousState.isNIGHT || currentState.lightAutomation != previousState.lightAutomation || currentState.ledInVal != previousState.ledInVal || currentState.ledOutVal != previousState.ledOutVal)
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
  }
}

void updateCurrentState()
{
  currentState.isNIGHT = digitalRead(pinLUMEN);
  currentState.temperatureValue = readTemperatureSensor();
  currentState.humidityValue = readHumiditySensor();
  currentState.gasValue = readGasSensor();
}

void updatePreviousState()
{
  previousState.isNIGHT = currentState.isNIGHT;
  previousState.temperatureValue = currentState.temperatureValue;
  previousState.humidityValue = currentState.humidityValue;
  previousState.gasValue = currentState.gasValue;
  previousState.lightAutomation = currentState.lightAutomation;
  previousState.ledOutVal = currentState.ledOutVal;
  previousState.ledInVal = currentState.ledInVal;
  previousState.isLocked = currentState.isLocked;
}

void compareStates()
{
  if (
      currentState.isNIGHT != previousState.isNIGHT ||
      currentState.temperatureValue != previousState.temperatureValue ||
      currentState.humidityValue != previousState.humidityValue ||
      currentState.gasValue != previousState.gasValue ||
      currentState.lightAutomation != previousState.lightAutomation ||
      currentState.ledOutVal != previousState.ledOutVal ||
      currentState.ledInVal != previousState.ledInVal ||
      currentState.isLocked != previousState.isLocked)
  {
    needToNotify = true;
  }
}

/*****************************************************/

String getStateMessage()
{

  // currentState.isNIGHT
  // currentState.temperatureValue
  // currentState.humidityValue
  // currentState.gasValue
  // currentState.lightAutomation
  // currentState.ledOutVal
  // currentState.ledInVal
  // currentState.isLocked

  String message = "{\"house\":{\"isNIGHT\":" + String(currentState.isNIGHT) + ",\"temperatureValue\":" + String(currentState.temperatureValue) + ",\"humidityValue\":" + String(currentState.humidityValue) + ",\"gasValue\":" + String(currentState.gasValue) + ",\"lightAutomation\":" + String(currentState.lightAutomation) + ",\"ledOutVal\":" + String(currentState.ledOutVal) + ",\"ledInVal\":" + String(currentState.ledInVal) + ",\"isLocked\":" + String(currentState.isLocked) + "}}";
  return message;
}

void notifyClients()
{
  String message = getStateMessage();
  ws.textAll(message);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  String message = (char *)data;
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    data[len] = 0;
    needToNotify = true;

    if (strcmp((char *)data, "lockDoor") == 0)
    {
      closeDoor();
    }

    if (strcmp((char *)data, "unlockDoor") == 0)
    {
      openDoor();
    }

    if (strcmp((char *)data, "automationOn") == 0)
    {
      turnAutomationON();
    }

    if (strcmp((char *)data, "automationOff") == 0)
    {
      turnAutomationOFF();
    }

    if (strcmp((char *)data, "turnLEDinON") == 0)
    {
      turnLEDinON();
    }
    if (strcmp((char *)data, "turnLEDinOFF") == 0)
    {
      turnLEDinOFF();
    }
    if (strcmp((char *)data, "turnLEDoutON") == 0)
    {
      turnLEDoutON();
    }
    if (strcmp((char *)data, "turnLEDoutOFF") == 0)
    {
      turnLEDoutOFF();
    }

    // if (message.indexOf("1s") >= 0)
    // {
    //   int messageInt = message.substring(2).toInt();
    //   if (homePosition)
    //   {
    //     long targetPosition = map(messageInt, 0, 100, 0, homePosition);
    //     handleMoveTo(targetPosition);

    //     Serial.printf("Target position from slider: %d, mapped: %ld.\n", messageInt, targetPosition);
    //   }
    //   else
    //   {
    //     Serial.println("Please set homeposition first!");
    //   }
    // }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    ws.text(client->id(), getStateMessage());
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

void initWebSocket()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

/*****************************************************/

void setup()
{
  Serial.begin(9600);
  Serial.println("Setup initialized!");

  pinMode(pinLUMEN, INPUT);   // LUMEN pin as input.
  pinMode(pinLEDin, OUTPUT);  // LED pin as output.
  pinMode(pinLEDout, OUTPUT); // LED pin as output.
  pinMode(pinGas, INPUT);

  /**************************************************/

  WiFi.mode(WIFI_AP_STA);
  WiFi.hostname(newHostname.c_str());
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  // Print ESP Local IP Address
  Serial.println(WiFi.localIP());

  initWebSocket();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", index_html); });

  // Start server
  server.begin();

  /**************************************************/

  myservo.attach(pinServo);
  dht.begin();
  Serial.println("Setup completed!");
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
  // readSensors();
  // algorithms();

  // Serial.printf("T=%f | H=%f | Gas=%d | Night=%d \n", currentState.temperatureValue, currentState.humidityValue, currentState.gasValue, currentState.isNIGHT);

  /************************************/

  ws.cleanupClients();
  EVERY_MS(300)
  {
    updateCurrentState();
    compareStates();
  }

  if (needToNotify)
  {
    LEDup();

    needToNotify = false;
    updatePreviousState();

    if (ws.count() > 0)
    {
      notifyClients();
    }
  }
  /************************************/
}