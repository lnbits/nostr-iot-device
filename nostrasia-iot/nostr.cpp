#include "nostr.h"

NostrEvent nostr;
NostrRelayManager nostrRelayManager;
NostrQueueProcessor nostrQueue;

String serialisedEventRequest;

NostrRequestOptions* eventRequestOptions;

const size_t capacity = JSON_OBJECT_SIZE(2) + 70;

char deviceSk[80] = DEVICE_SK;
char devicePk[80] = DEVICE_PK;
char relayString[80] = RELAY;

int socketDisconnectedCount = 0;

extern String lastPayload;

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

  String authors[] = {String(REMOTE_CONTROL_PK)};
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
  nostrRelayManager.setEventCallback("ok", okEvent);
  nostrRelayManager.setEventCallback("connected", relayConnectedEvent);
  nostrRelayManager.setEventCallback("disconnected", relayDisonnectedEvent);
  nostrRelayManager.setEventCallback(8000, iotIntentEvent);

  Serial.println("Connecting to Nostr relays");
  writeTextToTft("Connecting to Nostr relays");
  nostrRelayManager.connect();

}

void relayConnectedEvent(const std::string& key, const std::string& message) {
  socketDisconnectedCount = 0;
  Serial.println("Relay connected.");
  writeTextToTft("Relay connected.");
  
  Serial.print(F("Requesting events."));
  Serial.println(serialisedEventRequest);
  writeTextToTft("Requesting events.");

  nostrRelayManager.broadcastEvent(serialisedEventRequest);
}

void relayDisonnectedEvent(const std::string& key, const std::string& message) {
  Serial.println("Relay disconnected: ");
  writeTextToTft("Relay disconnected: ");
  socketDisconnectedCount++;
  // reboot after 3 socketDisconnectedCount subsequenet messages
  if(socketDisconnectedCount >= 3) {
    Serial.println("Too many socket disconnections. Restarting");
    writeTextToTft("Too many socket disconnections. Restarting");
    // restart device
    ESP.restart();
  }
}

/**
 * @brief Callback for when an OK event is received from a relay
 * 
 * @param key 
 * @param payload 
 */
void okEvent(const std::string& key, const char* payload) {
    if(lastPayload != payload) { // Prevent duplicate events from multiple relays triggering the same logic
      lastPayload = payload;
      Serial.println("payload is: ");
      Serial.println(payload);
      writeTextToTft("OK - " + String(payload));
    }
}
 
/**
 * @brief Callback for when an IoT intent event is received from a relay
 * 
 * @param key 
 * @param payload 
 */
void iotIntentEvent(const std::string& key, const char* payload) {
  if(lastPayload != payload) { // Prevent duplicate events from multiple relays triggering the same logic, as we are using multiple relays, this is likely to happen
    lastPayload = payload;
    Serial.println("ioT intent event:");
    Serial.println(payload);
    writeTextToTft("ioT intent event");
  }
}

/**
 * @brief Handle the last received payload
 * 
 * @param payload 
 */
void handlePayload(String payload) {
  // if lastpayload is set, then decypt it
  if(payload != "") {
    String message = nostr.decryptDm(DEVICE_SK, payload);
    Serial.println(message);
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
    // if temperature, then set the temperature
    if(strcmp(name, "temperature") == 0) {
      setTemperature(value);
    }
  }
}

/**
 * @brief Nostr loop actions
 * 
 */
void nostrLoop() {
  nostrRelayManager.loop();
  nostrRelayManager.broadcastEvents();
}