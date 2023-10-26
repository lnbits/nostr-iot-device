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
        delay(5);
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
        writeTextToTft("Lamp on");
    } else {
        analogWrite(ledPin, 0);
        writeTextToTft("Lamp off");
    }
}