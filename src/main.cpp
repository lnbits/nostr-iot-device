#include <Arduino.h>
#include <WiFi.h>
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
#include <ESP32Servo.h>
#include "wManager.h"

#include <ArduinoJson.h>

// freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BUZZER_PIN 2      // Connect the piezo buzzer to this GPIO pin.
#define CLICK_DURATION 20 // Duration in milliseconds.

#define PARAM_FILE "/elements.json"

Servo myservo;
int pos = 0;

int triggerAp = false;

bool lastInternetConnectionState = true;

int socketDisconnectedCount = 0;

int ledPin = 15; // Pin number where the LED is connected
extern int buttonPin; // Pin number where the button is connected
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

extern bool hasInternetConnection;

NostrRequestOptions* eventRequestOptions;

bool hasSentEvent = false;

bool isBuzzerEnabled = false;

extern char npubHexString[80];
extern char relayString[80];

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
uint16_t getRandomNum(uint16_t min, uint16_t max);
void loadSettings();
int64_t getAmountInSatoshis(const String &input);
String getBolt11InvoiceFromEvent(String jsonStr);
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

// Define the WiFi event callback function
void WiFiEvent(WiFiEvent_t event) {
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("Connected to WiFi and got an IP");
      click(225);
      delay(100);
      click(225);
      connectToNostrRelays();      
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi");
      // WiFi.begin(ssid, password); // Try to reconnect after getting disconnected
      break;
  }
}

void initNeck() {
 // move neck to pos
 // ta 
 myservo.write(90);
}

void moveNeck() {
  Serial.println("Moving neck");
  for (pos = 20; pos <= 140; pos += 1) {
    myservo.write(pos);
    delay(15);
  }
  for (pos = 140; pos >= 20; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
  for (pos = 20; pos <= 140; pos += 1) {
    myservo.write(pos);
    delay(15);
  }
  for (pos = 140; pos >= 20; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
  initNeck();
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
      moveNeck();
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
  int kinds[] = {8000};
  eventRequestOptions->kinds = kinds;
  eventRequestOptions->kinds_count = sizeof(kinds) / sizeof(kinds[0]);

  // String authors[] = {"c7c9bef6312c220e6de50897219904d67d272049eb251cefd0f2ff296f0698e7"};
  // eventRequestOptions->authors = authors;
  // eventRequestOptions->authors_count = sizeof(authors) / sizeof(authors[0]);

  eventRequestOptions->limit = 0;

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

String getBolt11InvoiceFromEvent(String jsonStr) {
  // Remove all JSON formatting characters
  String str = jsonStr.substring(1, jsonStr.length()-1); // remove the first and last square brackets
  str.replace("\\", ""); // remove all backslashes

  // Search for the "bolt11" substring
  int index = str.indexOf("bolt11");

  // Extract the value associated with "bolt11"
  String bolt11 = "";
  if (index != -1) {
    int start = index + 9; // the value of "bolt11" starts 9 characters after the substring index
    int end = start; // initialize the end index
    while (str.charAt(end) != '\"') {
      end++; // increment the end index until the closing double-quote is found
    }
    bolt11 = str.substring(start, end); // extract the value of "bolt11"
  }
  return bolt11;
}

/**
 * @brief Get the Amount In Satoshis from a lightning bol11 invoice
 * 
 * @param input 
 * @return int64_t 
 */
int64_t getAmountInSatoshis(const String &input) {
    int64_t number = -1;
    char multiplier = ' ';

    for (unsigned int i = 0; i < input.length(); ++i) {
        if (isdigit(input[i])) {
            number = 0;
            while (isdigit(input[i])) {
                number = number * 10 + (input[i] - '0');
                ++i;
            }
            for (unsigned int j = i; j < input.length(); ++j) {
                if (isalpha(input[j])) {
                    multiplier = input[j];
                    break;
                }
            }
            break;
        }
    }

    if (number == -1 || multiplier == ' ') {
        return -1;
    }

    int64_t satoshis = number;

    switch (multiplier) {
        case 'm':
            satoshis *= 100000; // 0.001 * 100,000,000
            break;
        case 'u':
            satoshis *= 100; // 0.000001 * 100,000,000
            break;
        case 'n':
            satoshis /= 10; // 0.000000001 * 100,000,000
            break;
        case 'p':
            satoshis /= 10000; // 0.000000000001 * 100,000,000
            break;
        default:
            return -1;
    }

    return satoshis;
}


uint16_t getRandomNum(uint16_t min, uint16_t max) {
  uint16_t rand  = (esp_random() % (max - min + 1)) + min;
  Serial.println("Random number: " + String(rand));
  return rand;
}

void iotIntentEvent(const std::string& key, const char* payload) {
    // if(lastPayload != payload) { // Prevent duplicate events from multiple relays triggering the same logic, as we are using multiple relays, this is likely to happen
      lastPayload = payload;
      queueMovement(1);
    // }
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

  myservo.setPeriodHertz(50);    // Standard 50Hz servo
  myservo.attach(13, 500, 2400); // Pin 13 with a pulse width range of 500 to 2400

  pinMode(BUZZER_PIN, OUTPUT); // Set the buzzer pin as an output.
  click(225);

  FlashFS.begin(FORMAT_ON_FAIL);
  // init spiffs
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  zapMutex = xSemaphoreCreateMutex();

  buttonPin = 4;
  // Set the button pin as INPUT
  pinMode(buttonPin, INPUT_PULLUP);

  randomSeed(analogRead(0)); // Seed the random number generator

  // start lamp control task
  xTaskCreatePinnedToCore(
    lampControlTask,   /* Task function. */
    "lampControlTask",     /* String with name of task. */
    5000,            /* Stack size in bytes. */
    NULL,             /* Parameter passed as input of the task */
    2,                /* Priority of the task. */
    NULL,             /* Task handle. */
    1);               /* Core where the task should run */

  WiFi.onEvent(WiFiEvent);
  init_WifiManager();

  createNip91IntentReq();

   if(hasInternetConnection) {
    Serial.println("Has internet connection. Connectring to relays");
    connectToNostrRelays();
   }

  // Set the LED to the desired intensity
  analogWrite(ledPin, lightBrightness);
  
  // moveNeck();

}

bool lastInternetConnectionCheckTime = 0;

void loop() {
  // send ping to Quad9 9.9.9.9 every 10 seconds to check for internet connection
  if (millis() - lastInternetConnectionCheckTime > 10000) {
    if(WiFi.status() == WL_CONNECTED) {
      IPAddress ip(9,9,9,9);  // Quad9 DNS
      bool ret = Ping.ping(ip);
      if(ret) {
        if(!lastInternetConnectionState) {
          Serial.println("Internet connection has come back! :D");
          // reboot
          ESP.restart();
        }
        lastInternetConnectionState = true;
      } else {
        lastInternetConnectionState = false;
      }
    }
  }

  nostrRelayManager.loop();
  nostrRelayManager.broadcastEvents();

  // reboot every hour
  if (millis() > 3600000) {
    Serial.println("Rebooting");
    ESP.restart();
  }

}