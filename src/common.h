#pragma once

#include <TFT_eSPI.h>
#include <SPI.h>

void initTft();
void writeTextToTft(String text, int size = 2);

void initLamp();
void controlLamp(int state);