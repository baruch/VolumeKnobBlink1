#pragma once

#include <Arduino.h>
#include <PluggableUSB.h>

#include <WS2812.h>

#define blink1_ver_major 2
#define blink1_ver_minor 3

class Blink1HID : public PluggableUSBModule
{
private:
	uint8_t epType[1];

	uint8_t protocol;
	uint8_t idle;

	WS2812 led;
	int playing;

protected:
    // Implementation of the PUSBListNode
    int getInterface(uint8_t* interfaceCount);
    int getDescriptor(USBSetup& setup);
    bool setup(USBSetup& setup);
	uint8_t getShortName(char*);

	void handleMessage(uint8_t *msgbuf, uint16_t length);
	void rgb_setCurr(uint8_t *buf);
	void rgb_setDest(uint8_t *buf, uint16_t steps);

public:
    Blink1HID(int led_pin);
    void wakeupHost(void);

};
