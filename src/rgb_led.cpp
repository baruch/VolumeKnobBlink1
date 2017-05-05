#include "rgb_led.h"

#include <WS2812.h>

WS2812 led(1); // Only one led

rgbint_t dest100x;
rgbint_t step100x;
rgbint_t curr100x;
int stepcnt;

static void led_update()
{
        led.setRGB(0, curr100x.r/100, curr100x.g/100, curr100x.b/100);
        led.sync();
}

void rgb_updateStep()
{
        if (!stepcnt)
                return;

        stepcnt--;

        if( stepcnt ) {
                curr100x.r += step100x.r;
                curr100x.g += step100x.g;
                curr100x.b += step100x.b;
        } else {
                curr100x.r = dest100x.r;
                curr100x.g = dest100x.g;
                curr100x.b = dest100x.b;
        }

        led_update();
}

void rgb_setCurr(rgb_t *newcolor)
{
        curr100x.r = newcolor->r * 100;
        curr100x.g = newcolor->g * 100;
        curr100x.b = newcolor->b * 100;

        dest100x.r = curr100x.r;
        dest100x.g = curr100x.g;
        dest100x.b = curr100x.b;

        stepcnt = 0;

        led_update();
}

void rgb_setDest(rgb_t *newcolor, int steps)
{
        dest100x.r = newcolor->r * 100;
        dest100x.g = newcolor->g * 100;
        dest100x.b = newcolor->b * 100;

        stepcnt = steps + 1;

        step100x.r = (dest100x.r - curr100x.r ) / steps;
        step100x.g = (dest100x.g - curr100x.g ) / steps;
        step100x.b = (dest100x.b - curr100x.b ) / steps;
}

void rgb_init(int led_pin)
{
        led.setOutput(led_pin);
        led_update();
}
