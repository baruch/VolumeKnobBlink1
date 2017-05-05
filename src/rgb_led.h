#pragma once

#include <Arduino.h>

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} rgb_t;

typedef struct {
	int r;
	int g;
	int b;
} rgbint_t;

typedef struct {
    rgb_t color;
    uint16_t dmillis; // hundreths of a sec
} patternline_t;

void rgb_init(int led_pin);
void rgb_updateStep(void);
void rgb_setCurr(rgb_t *newcolor);
void rgb_setDest(rgb_t *newcolor, int steps);
