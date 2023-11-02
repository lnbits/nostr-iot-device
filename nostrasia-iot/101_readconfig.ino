String getJsonValue(JsonDocument &doc, const char* name)
{
    for (JsonObject elem : doc.as<JsonArray>()) {
        if (strcmp(elem["name"], name) == 0) {
            String value = elem["value"].as<String>();
            return value;
        }
    }
    return "";  // return empty string if not found
}

void readConfig(JsonDocument &doc)
{
    File paramFile = FlashFS.open(PARAM_FILE, "r");
    if (paramFile)
    {
        DeserializationError error = deserializeJson(doc, paramFile.readString());
        paramFile.close();
        if(error){
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.c_str());
        }
    }
}
