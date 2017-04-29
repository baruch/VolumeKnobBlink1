#include <Arduino.h>
#include <Rotary.h>
#include <Button.h>
#include <WS2812.h>
#include <HID-Project.h>

#define PIN_ROTARY_ENCODER_A 4
#define PIN_ROTARY_ENCODER_B 5

static Rotary knob(PIN_ROTARY_ENCODER_A, PIN_ROTARY_ENCODER_B);
static Button knob_switch;
static WS2812 led(1);
#define hidKeyboard Consumer

static int is_muted;

void setup()
{
        // Start keyboard emulation
        hidKeyboard.begin();

        //Serial.begin(115200);

        // Initialize play/pause switch
        pinMode(3, INPUT_PULLUP);
        knob_switch.initialize(3);
        is_muted = 0;

        // Initialize led
        led.setOutput(2);
        led.setRGB(0, 0, 128, 0);
        led.sync();
}

void loop()
{
        knob_switch.update();
        unsigned char knob_change = knob.process();

        if (knob_change == DIR_CW) {
                // Increase volume
                hidKeyboard.write(MEDIA_VOLUME_UP);
        } else if (knob_change == DIR_CCW) {
                // Decrease volume
                hidKeyboard.write(MEDIA_VOLUME_DOWN);
        }

        if (knob_switch.isPressed()) {
                is_muted = 1 - is_muted;
                //hidKeyboard.println("Test");
                if (is_muted) {
                        //Serial.println("Muted");
                        led.setRGB(0, 128, 0, 0);
                        hidKeyboard.write(MEDIA_VOLUME_MUTE);
                        //hidKeyboard.write(MEDIA_PLAY_PAUSE);
                } else {
                        //Serial.println("Playing");
                        led.setRGB(0, 0, 128, 0);
                        hidKeyboard.write(MEDIA_VOLUME_MUTE);
                        //hidKeyboard.write(MEDIA_PLAY_PAUSE);
                }
                led.sync();
        }
}
