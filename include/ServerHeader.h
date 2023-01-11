#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SiteHeader.h>

// const char *ssid = "ProiectSCR";
// const char *password = "studentscr";

const char *ssid = "BEAMLOGIC";
const char *password = "1234567890123";

String newHostname = "Wemos-Smart-House";
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");