#include <Arduino.h>
#include "settings.h"
#include <WiFiClientSecure.h>
#include "time.h"
#include <NostrEvent.h>
#include <NostrRelayManager.h>
#include <NostrRequestOptions.h>
#include "Bitcoin.h"
#include "Hash.h"
#include <math.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <ArduinoJson.h>


#define ADC_EN              14  //ADC_EN is the ADC detection enable port
#define ADC_PIN             34
#define BUTTON_1            35
#define BUTTON_2            0

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library

int pos = 0;

bool lastInternetConnectionState = true;

int socketDisconnectedCount = 0;

int ledPin = 15; // Pin number where the LED is connected
int lightBrightness = 50; // The brightness of the LED (0-255)

SemaphoreHandle_t zapMutex;

NostrEvent nostr;
NostrRelayManager nostrRelayManager;
NostrQueueProcessor nostrQueue;

String serialisedEventRequest;

NostrRequestOptions* eventRequestOptions;

bool hasSentEvent = false;

bool isBuzzerEnabled = false;

char deviceSk[80] = DEVICE_SK;
char devicePk[80] = DEVICE_PK;
char relayString[80] = RELAY;

fs::SPIFFSFS &FlashFS = SPIFFS;
#define FORMAT_ON_FAIL true

// define funcs
void iotIntentEvent(const std::string& key, const char* payload);
void okEvent(const std::string& key, const char* payload);
void nip01Event(const std::string& key, const char* payload);
void relayConnectedEvent(const std::string& key, const std::string& message);
void relayDisonnectedEvent(const std::string& key, const std::string& message);
void createIntentReq();
void connectToNostrRelays();

void writeTextToTft(String text, int size = 2) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(size);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(text,  tft.width() / 2, tft.height() / 2 );
}

/**
 * @brief Create a Zap Event Request object
 * 
 */
void createIntentReq() {
  // Create the REQ
  eventRequestOptions = new NostrRequestOptions();
  // Populate kinds
  int kinds[] = {8000};
  eventRequestOptions->kinds = kinds;
  eventRequestOptions->kinds_count = sizeof(kinds) / sizeof(kinds[0]);

  String authors[] = {"d58d5dc2abdef2195532b0940d56bc44c693b48084bf11d0bb70035510c9e6b5"};
  eventRequestOptions->authors = authors;
  eventRequestOptions->authors_count = sizeof(authors) / sizeof(authors[0]);

  eventRequestOptions->limit = 1;

  // We store this here for sending this request again if a socket reconnects
  serialisedEventRequest = "[\"REQ\", \"" + nostrRelayManager.getNewSubscriptionId() + "\"," + eventRequestOptions->toJson() + "]";

  delete eventRequestOptions;
}

/**
 * @brief Connect to the Nostr relays
 * 
 */
void connectToNostrRelays() {
  // first disconnect from all relays
  nostrRelayManager.disconnect();
  Serial.println("Requesting Zap notifications");

  // split relayString by comma into vector
  std::vector<String> relays;
  String relayStringCopy = String(relayString);
  int commaIndex = relayStringCopy.indexOf(",");
  while (commaIndex != -1) {
    relays.push_back(relayStringCopy.substring(0, commaIndex));
    relayStringCopy = relayStringCopy.substring(commaIndex + 1);
    commaIndex = relayStringCopy.indexOf(",");
  }
  // add last item after last comma
  if (relayStringCopy.length() > 0) {
    relays.push_back(relayStringCopy);
  }

  // no need to convert to char* anymore
  nostr.setLogging(false);
  nostrRelayManager.setRelays(relays);
  nostrRelayManager.setMinRelaysAndTimeout(1,10000);

  // Set some event specific callbacks here
  Serial.println("Setting callbacks");
  nostrRelayManager.setEventCallback("ok", okEvent);
  nostrRelayManager.setEventCallback("connected", relayConnectedEvent);
  nostrRelayManager.setEventCallback("disconnected", relayDisonnectedEvent);
  nostrRelayManager.setEventCallback(8000, iotIntentEvent);
  // nostrRelayManager.setEventCallback(4, iotIntentEvent);

  Serial.println("connecting");
  nostrRelayManager.connect();

}

String lastPayload = "";

void relayConnectedEvent(const std::string& key, const std::string& message) {
  socketDisconnectedCount = 0;
  Serial.println("Relay connected: ");
  
  Serial.print(F("Requesting events:"));
  Serial.println(serialisedEventRequest);

  nostrRelayManager.broadcastEvent(serialisedEventRequest);
}

void relayDisonnectedEvent(const std::string& key, const std::string& message) {
  Serial.println("Relay disconnected: ");
  socketDisconnectedCount++;
  // reboot after 3 socketDisconnectedCount subsequenet messages
  if(socketDisconnectedCount >= 3) {
    Serial.println("Too many socket disconnections. Restarting");
    // restart device
    ESP.restart();
  }
}

void okEvent(const std::string& key, const char* payload) {
    if(lastPayload != payload) { // Prevent duplicate events from multiple relays triggering the same logic
      lastPayload = payload;
      Serial.println("payload is: ");
      Serial.println(payload);
    }
}

void nip01Event(const std::string& key, const char* payload) {
    if(lastPayload != payload) { // Prevent duplicate events from multiple relays triggering the same logic
      lastPayload = payload;
      // We can convert the payload to a StaticJsonDocument here and get the content
      StaticJsonDocument<1024> eventJson;
      deserializeJson(eventJson, payload);
      String pubkey = eventJson[2]["pubkey"].as<String>();
      String content = eventJson[2]["content"].as<String>();
      Serial.println(pubkey + ": " + content);
    }
}


void iotIntentEvent(const std::string& key, const char* payload) {
  if(lastPayload != payload) { // Prevent duplicate events from multiple relays triggering the same logic, as we are using multiple relays, this is likely to happen
    lastPayload = payload;
    Serial.println("payload is: ");
    Serial.println(payload);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("boot");

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(3);

  writeTextToTft("booting");

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

  // Set the LED to the desired intensity
  analogWrite(ledPin, lightBrightness);

}

void controlLamp(int state) {
  if(state == 1) {
    analogWrite(ledPin, 255);
  } else {
    analogWrite(ledPin, 0);
  }
}

bool lastInternetConnectionCheckTime = 0;


const size_t capacity = JSON_OBJECT_SIZE(2) + 70;

void loop() {
  nostrRelayManager.loop();
  nostrRelayManager.broadcastEvents();

  // reboot every hour
  if (millis() > 3600000) {
    Serial.println("Rebooting");
    ESP.restart();
  }

  // if lastpayload is set, then decrpyt it
  if(lastPayload != "") {
    String message = nostr.decryptDm("c63fbf2c708b8dcd9049ca61f01b48e9b19d023c3363fd2797ee8842dc48c45e", lastPayload);
    Serial.println(message);
    lastPayload = "";
    // Create a StaticJsonDocument object
    StaticJsonDocument<capacity> doc;

    // Deserialize the JSON string
    DeserializationError error = deserializeJson(doc, message);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    // Get the values
    const char* name = doc["name"];    // "light"
    int value = doc["value"];          // 1

    // Print the values
    Serial.println(name);
    Serial.println(value);
    // if light, then set the light
    if(strcmp(name, "light") == 0) {
      controlLamp(value);
    }
    // if temperature, show on the screen
    if(strcmp(name, "temperature") == 0) {
      writeTextToTft(String(value), 4);
    }
  }

}