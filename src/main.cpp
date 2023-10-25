#include <Arduino.h>
#include "settings.h"
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include "time.h"
#include <NostrEvent.h>
#include <NostrRelayManager.h>
#include <NostrRequestOptions.h>
#include <Wire.h>
#include "Bitcoin.h"
#include "Hash.h"
#include <esp_random.h>
#include <math.h>
#include <SPIFFS.h>
#include <vector>
#include <ESP32Ping.h>
#include <TFT_eSPI.h>
#include <SPI.h>

#include <ArduinoJson.h>

// freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BUZZER_PIN 2      // Connect the piezo buzzer to this GPIO pin.
#define CLICK_DURATION 20 // Duration in milliseconds.

#define PARAM_FILE "/elements.json"

#define ADC_EN              14  //ADC_EN is the ADC detection enable port
#define ADC_PIN             34
#define BUTTON_1            35
#define BUTTON_2            0

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library

int pos = 0;

int triggerAp = false;

bool lastInternetConnectionState = true;

int socketDisconnectedCount = 0;

int ledPin = 15; // Pin number where the LED is connected
int minFlashDelay = 100; // Minimum delay between flashes (in milliseconds)
int maxFlashDelay = 5000; // Maximum delay between flashes (in milliseconds)
int lightBrightness = 50; // The brightness of the LED (0-255)

SemaphoreHandle_t zapMutex;

// create a vector for storing zap amount for the flash queue
std::vector<int> zapAmountsFlashQueue;

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
void click(int period);
void configureAccessPoint();
void initWiFi();
bool whileCP(void);
void queueMovement(int zapAmountSats);
unsigned long getUnixTimestamp();
void iotIntentEvent(const std::string& key, const char* payload);
void okEvent(const std::string& key, const char* payload);
void nip01Event(const std::string& key, const char* payload);
void relayConnectedEvent(const std::string& key, const std::string& message);
void relayDisonnectedEvent(const std::string& key, const std::string& message);
void loadSettings();
void createNip91IntentReq();
void connectToNostrRelays();

#define BUTTON_PIN 0 // change this to the pin your button is connected to
#define DOUBLE_TAP_DELAY 250 // delay for double tap in milliseconds

volatile unsigned long lastButtonPress = 0;
volatile bool doubleTapDetected = false;

void IRAM_ATTR handleButtonInterrupt() {
  unsigned long now = millis();
  if (now - lastButtonPress < DOUBLE_TAP_DELAY) {
    doubleTapDetected = true;
  }
  lastButtonPress = now;
}

void writeTextToTft(String text) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(text,  tft.width() / 2, tft.height() / 2 );
}

//free rtos task for lamp control
void lampControlTask(void *pvParameters) {
  Serial.println("Starting lamp control task");

  for(;;) {
    if(!lastInternetConnectionState) {
      Serial.println("No internet connection. Not doing anything");
    }

    // watch for lamp state and do as needed
    if (zapAmountsFlashQueue.size() > 0) {
      Serial.println("Neck movement queue size: " + String(zapAmountsFlashQueue.size()));
      // take the item off the queue
      xSemaphoreTake(zapMutex, portMAX_DELAY);
      int zapAmount = zapAmountsFlashQueue[0];
      zapAmountsFlashQueue.erase(zapAmountsFlashQueue.begin());
      xSemaphoreGive(zapMutex);
      // moveNeck();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Create a Zap Event Request object
 * 
 */
void createNip91IntentReq() {
  // Create the REQ
  eventRequestOptions = new NostrRequestOptions();
  // Populate kinds
  int kinds[] = {4};
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
  nostr.setLogging(true);
  nostrRelayManager.setRelays(relays);
  nostrRelayManager.setMinRelaysAndTimeout(1,10000);

  // Set some event specific callbacks here
  Serial.println("Setting callbacks");
  nostrRelayManager.setEventCallback("ok", okEvent);
  nostrRelayManager.setEventCallback("connected", relayConnectedEvent);
  nostrRelayManager.setEventCallback("disconnected", relayDisonnectedEvent);
  nostrRelayManager.setEventCallback(8000, iotIntentEvent);
  nostrRelayManager.setEventCallback(4, iotIntentEvent);

  Serial.println("connecting");
  nostrRelayManager.connect();

}


void queueMovement(int timesToMove) {
  int flashCount = 1;
  // set flash count length of the number in the zap amount
  if (timesToMove > 0) {
    flashCount = floor(log10(timesToMove)) + 1;
  }

  // push to the flash queue
  xSemaphoreTake(zapMutex, portMAX_DELAY);
  zapAmountsFlashQueue.push_back(flashCount);
  xSemaphoreGive(zapMutex);

}

unsigned long getUnixTimestamp() {
  time_t now;
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return 0;
  } else {
    Serial.println("Got timestamp of " + String(now));
  }
  time(&now);
  return now;
}

String lastPayload = "";

void relayConnectedEvent(const std::string& key, const std::string& message) {
  socketDisconnectedCount = 0;
  Serial.println("Relay connected: ");

  click(225);
  delay(100);
  click(225);
  delay(100);
  click(225);
  
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

      nostr.setLogging(true);
      String dmMessage = nostr.decryptDm(deviceSk, payload);
      // Serial.println("message is: ");
      // Serial.println(dmMessage);
      queueMovement(1);
    }
}

void click(int period)
{
  if(!isBuzzerEnabled) {
    return;
  }
  for (int i = 0; i < CLICK_DURATION; i++)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(period); // Half period of 1000Hz tone.
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(period); // Other half period of 1000Hz tone.
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("boot");

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(3);

  // get status of button1 and 2
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  // digital read
  int button1State = digitalRead(BUTTON_1);
  int button2State = digitalRead(BUTTON_2);

  writeTextToTft("booting: " + String(button1State) + " " + String(button2State));

  // connect to wifi using standard arduino method with ssid and password
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
    writeTextToTft("Connecting to WiFi..");
  }
  writeTextToTft("Connected to WiFi");
  

  pinMode(BUZZER_PIN, OUTPUT); // Set the buzzer pin as an output.
  click(225);

  FlashFS.begin(FORMAT_ON_FAIL);
  // init spiffs
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  zapMutex = xSemaphoreCreateMutex();

  randomSeed(analogRead(0)); // Seed the random number generator

  // start lamp control task
  // xTaskCreatePinnedToCore(
  //   lampControlTask,   /* Task function. */
  //   "lampControlTask",     /* String with name of task. */
  //   5000,            /* Stack size in bytes. */
  //   NULL,             /* Parameter passed as input of the task */
  //   2,                /* Priority of the task. */
  //   NULL,             /* Task handle. */
  //   1);               /* Core where the task should run */

  createNip91IntentReq();

  connectToNostrRelays();

  // Set the LED to the desired intensity
  analogWrite(ledPin, lightBrightness);
  
  // moveNeck();

}

bool lastInternetConnectionCheckTime = 0;

void loop() {
  nostrRelayManager.loop();
  nostrRelayManager.broadcastEvents();

  // reboot every hour
  if (millis() > 3600000) {
    Serial.println("Rebooting");
    ESP.restart();
  }

}