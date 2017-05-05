#include <Arduino.h>
#include <Rotary.h>
#include <Button.h>
#include <WS2812.h>

#include "blink1hid.h"
#include "rgb_led.h"

#define PIN_WS2812_LED 2
#define PIN_ROTARY_SWITCH 3
#define PIN_ROTARY_ENCODER_A 4
#define PIN_ROTARY_ENCODER_B 5

static Rotary knob(PIN_ROTARY_ENCODER_A, PIN_ROTARY_ENCODER_B);
static Button knob_switch;
static Blink1HID blink1;

static long last_led_update_ms;

void setup()
{
        // Start keyboard emulation
        blink1.begin();

        // Init led
        rgb_init(PIN_WS2812_LED);

        // Initialize play/pause switch
        pinMode(PIN_ROTARY_SWITCH, INPUT_PULLUP);
        knob_switch.initialize(PIN_ROTARY_SWITCH);

        last_led_update_ms = millis();
}

void loop()
{
        long now = millis();
        if (now - last_led_update_ms >= 10) {
                last_led_update_ms = now;
                rgb_updateStep();
        }

        knob_switch.update();
        unsigned char knob_change = knob.process();

        if (knob_change == DIR_CW) {
                // Increase volume
                blink1.write(MEDIA_VOLUME_UP);
        } else if (knob_change == DIR_CCW) {
                // Decrease volume
                blink1.write(MEDIA_VOLUME_DOWN);
        }

        if (knob_switch.isPressed()) {
                blink1.write(MEDIA_VOLUME_MUTE);
        }
}
