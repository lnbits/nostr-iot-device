#include "settings.h"
#include "common.h"
#include "nostr.h"

String lastPayload = "";
int ledPin = 27;

void setup() {
  Serial.begin(115200);
  Serial.println("boot");

  initTft();
  initLamp();
  writeTextToTft("Booting");

  // connect to wifi using standard arduino method with ssid and password
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
    writeTextToTft("Connecting to WiFi..");
  }
  writeTextToTft("Connected to WiFi");

  createIntentReq();
  connectToNostrRelays();
}

void loop() {
  nostrLoop();

  // reboot every hour. Helps with memory leaks and sometimes, relays disconnect but dont trigger the disconnect event
  if (millis() > 3600000) {
    Serial.println("Rebooting");
    ESP.restart();
  }

  if(lastPayload != "") {
    handlePayload(lastPayload);
    lastPayload = "";
  }

}
