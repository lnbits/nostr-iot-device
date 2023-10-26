#include "common.h"

#define ADC_EN              14  //ADC_EN is the ADC detection enable port
#define ADC_PIN             34
#define BUTTON_1            35
#define BUTTON_2            0

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library

extern int ledPin; // Pin number where the LED is connected

/**
 * @brief Initialise the TFT screen
 * 
 */
void initTft() {
    tft.init();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(3);
}

/**
 * @brief Write text to the TFT screen
 * 
 * @param text 
 * @param size 
 */
void writeTextToTft(String text, int size) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(size);
    tft.setTextWrap(true);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(text,  tft.width() / 2, tft.height() / 2 );
}

/**
 * @brief Initialise the lamp
 * 
 */
void initLamp() {
    // do a nice cycle of the lamp to show it's working
    for (int i = 0; i < 255; i++) {
        analogWrite(ledPin, i);
        delay(2);
    }
    // and down again
    for (int i = 255; i > 0; i--) {
        analogWrite(ledPin, i);
        delay(2);
    }
}

/**
 * @brief Control the lamp
 * 
 * @param state 1 for on, 0 for off
 */
void controlLamp(int state) {
    if(state == 1) {
        analogWrite(ledPin, 255);
        writeTextToTft("Lamp on", 3);
    } else {
        analogWrite(ledPin, 0);
        writeTextToTft("Lamp off", 3);
    }
}

/**
 * @brief Set the Temperature
 * 
 * @param value 
 */
void setTemperature(int value) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(2);
    tft.drawString("Temperature set to",  tft.width() / 2, (tft.height() / 2) - 20 );
    tft.setTextSize(5);
    tft.drawString(String(value) + "C",  tft.width() / 2, (tft.height() / 2) + 20 );
}