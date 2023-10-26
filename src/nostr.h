#pragma once

#include <NostrEvent.h>
#include <NostrRelayManager.h>
#include <NostrRequestOptions.h>
#include <ArduinoJson.h>
#include "Bitcoin.h"
#include "Hash.h"
#include <math.h>

#include "settings.h"
#include "common.h"

void createIntentReq();
void connectToNostrRelays();
void handlePayload(String payload);
void iotIntentEvent(const std::string& key, const char* payload);
void okEvent(const std::string& key, const char* payload);
void relayConnectedEvent(const std::string& key, const std::string& message);
void relayDisonnectedEvent(const std::string& key, const std::string& message);
void handlePayload(String payload);
void nostrLoop();