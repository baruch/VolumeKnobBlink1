#include <Arduino.h>
#include "blink1hid.h"
#include "HID.h"
#include "HID-Settings.h"

#define Serial Serial1

static const uint8_t hidReportDescriptorBlink1[] PROGMEM = {
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x95, 8,                       //   REPORT_COUNT (8)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};


Blink1HID::Blink1HID(int led_pin) : PluggableUSBModule(1, 1, epType),
                   protocol(HID_REPORT_PROTOCOL), idle(1),
                   led(1)
{
        epType[0] = EP_TYPE_INTERRUPT_IN;
        PluggableUSB().plug(this);
        led.setOutput(led_pin);
}

int Blink1HID::getInterface(uint8_t* interfaceCount)
{
        *interfaceCount += 1; // uses 1
        HIDDescriptor hidInterface = {
                D_INTERFACE(pluggedInterface, 1, USB_DEVICE_CLASS_HUMAN_INTERFACE, HID_SUBCLASS_NONE, HID_PROTOCOL_NONE),
                D_HIDREPORT(sizeof(hidReportDescriptorBlink1)), // TODO: to add keyboard, total the size of the descriptors
                D_ENDPOINT(USB_ENDPOINT_IN(pluggedEndpoint), USB_ENDPOINT_TYPE_INTERRUPT, USB_EP_SIZE, 0x01)
        };
        return USB_SendControl(0, &hidInterface, sizeof(hidInterface));
}


int Blink1HID::getDescriptor(USBSetup& setup)
{
        // Check if this is a HID Class Descriptor request
        if (setup.bmRequestType != REQUEST_DEVICETOHOST_STANDARD_INTERFACE) { return 0; }
        if (setup.wValueH != HID_REPORT_DESCRIPTOR_TYPE) { return 0; }

        // In a HID Class Descriptor wIndex contains the interface number
        if (setup.wIndex != pluggedInterface) { return 0; }

        // Reset the protocol on reenumeration. Normally the host should not assume the state of the protocol
        // due to the USB specs, but Windows and Linux just assumes its in report mode.
        protocol = HID_REPORT_PROTOCOL;

        int total = 0;
        total += USB_SendControl(TRANSFER_PGM, hidReportDescriptorBlink1, sizeof(hidReportDescriptorBlink1));
        // TODO: To add keyboard send here the hid report for it, add it to total
        return total;
}

uint8_t Blink1HID::getShortName(char *name)
{
        name[0] = 'N';
        name[1] = 'o';
        name[2] = 'n';
        name[3] = 't';
        name[4] = 'h';
        name[5] = 'n';
        name[6] = 'g';
        return 7;
}

/*
int Blink1HID::SendReport(uint8_t id, const void* data, int len)
{
        auto ret = USB_Send(pluggedEndpoint, &id, 1);
        if (ret < 0) return ret;
        auto ret2 = USB_Send(pluggedEndpoint | TRANSFER_RELEASE, data, len);
        if (ret2 < 0) return ret2;
        return ret + ret2;
}
*/

bool Blink1HID::setup(USBSetup& setup)
{
        if (pluggedInterface != setup.wIndex) {
                return false;
        }

        uint8_t request = setup.bRequest;
        uint8_t requestType = setup.bmRequestType;

        if (requestType == REQUEST_DEVICETOHOST_CLASS_INTERFACE)
        {
                if (request == HID_GET_REPORT) {
                        // TODO: HID_GetReport();
                        return true;
                }
                if (request == HID_GET_PROTOCOL) {
                        // TODO: Send8(protocol);
                        return true;
                }
                if (request == HID_GET_IDLE) {
                        // TODO: Send8(idle);
                }
        }

        if (requestType == REQUEST_HOSTTODEVICE_CLASS_INTERFACE)
        {
                if (request == HID_SET_PROTOCOL) {
                        // The USB Host tells us if we are in boot or report mode.
                        // This only works with a real boot compatible device.
                        protocol = setup.wValueL;
                        return true;
                }
                if (request == HID_SET_IDLE) {
                        idle = setup.wValueL;
                        return true;
                }
                if (request == HID_SET_REPORT)
                {
                        //uint8_t reportID = setup.wValueL;
                        uint16_t length = setup.wLength;

                        // Make sure to not read more data than USB_EP_SIZE.
                        if (length > USB_EP_SIZE)
                                return false;

                        Serial.print("set report ");
                        Serial.println(length);

                        uint8_t data[length];
                        USB_RecvControl(data, length);
                        handleMessage(data, length);
                        return true;
                }
        }

        return false;
}

void Blink1HID::rgb_setCurr(uint8_t *buf)
{
        Serial.println(buf[0]);
        Serial.println(buf[1]);
        Serial.println(buf[2]);

        led.setRGB(0, buf[0], buf[1], buf[2]);
        led.sync();
}

void Blink1HID::rgb_setDest(uint8_t *buf, uint16_t steps)
{
        rgb_setCurr(buf);
}

void Blink1HID::handleMessage(uint8_t *msgbuf, uint16_t length)
{
        Serial.println("report");
        Serial.println(*msgbuf);

    uint8_t* msgbufp = msgbuf+1;  // skip over report id

    uint8_t cmd = msgbufp[0];

    Serial.print("cmd: ");
    Serial.println(cmd);

    // fade to RGB color
    // command {'c', r,g,b, th,tl, 0,0 } = fade to RGB color over time t
    // where time 't' is a number of 10msec ticks
    //
    if (cmd == 'c' ) {
            Serial.println("color fade");
        uint8_t* c = msgbufp+1; // msgbuf[1],msgbuf[2],msgbuf[3]
        uint16_t t = (msgbufp[4] << 8) | msgbufp[5]; // msgbuf[4],[5]
        playing = 0;
        rgb_setDest( c, t );
    }
    // set RGB color immediately  - {'n', r,g,b, 0,0,0,0 }
    //
    else if( cmd == 'n' ) {
            Serial.println("color now");
        uint8_t* c = (msgbufp+1);
        rgb_setDest( c, 0 );
        rgb_setCurr( c );
    }
    // play/pause, with position  - {'p', p, 0,0, 0,0,0,0}
    //
    else if( cmd == 'p' ) {
#if 0
        playing = msgbufp[1];
        playpos = msgbufp[2];
        startPlaying();
#endif
    }
    // write color pattern entry - {'P', r,g,b, th,tl, p, 0}
    //
    else if( cmd == 'P' ) {
#if 0
        // was doing this copy with a cast, but broke it out for clarity
        patternline_t ptmp;
        ptmp.color.r = msgbufp[1];
        ptmp.color.g = msgbufp[2];
        ptmp.color.b = msgbufp[3];
        ptmp.dmillis = ((uint16_t)msgbufp[4] << 8) | msgbufp[5];
        uint8_t p = msgbufp[6];
        if( p >= patt_max ) p = 0;
        // save pattern line to RAM
        memcpy( &pattern[p], &ptmp, sizeof(patternline_t) );
        eeprom_write_block( &pattern[p], &ee_pattern[p], sizeof(patternline_t));
#endif
    }
    // read color pattern entry - {'R', 0,0,0, 0,0, p,0}
    //
    else if( cmd == 'R' ) {
#if 0
        uint8_t p = msgbufp[6];
        if( p >= patt_max ) p = 0;
        patternline_t ptmp ;
        eeprom_read_block( &ptmp, &ee_pattern[p], sizeof(patternline_t));
        msgbuf[2] = ptmp.color.r;
        msgbuf[3] = ptmp.color.g;
        msgbuf[4] = ptmp.color.b;
        msgbuf[5] = (ptmp.dmillis >> 8);
        msgbuf[6] = (ptmp.dmillis & 0xff);
#endif
    }
    // read eeprom byte - { 'e', addr, 0,0, 0,0,0,0}
    //
    else if( cmd == 'e' ) {
#if 0
        uint8_t addr = msgbufp[1];
        uint8_t val = eeprom_read_byte( (uint8_t*)(uint16_t)addr ); // dumb
        msgbufp[2] = val;  // put read byte in output buff
#endif
    }
    // write eeprom byte - { 'E', addr,val, 0, 0,0,0,0}
    //
    else if( cmd == 'E' ) {
#if 0
        uint8_t addr = msgbufp[1];
        uint8_t val  = msgbufp[2];
        if( addr > 0 ) {  // don't let overwrite osccal value
            eeprom_write_byte( (uint8_t*)(uint16_t)addr, val ); // dumb
        }
#endif
    }
    // servermode tickle - {'D', {1/0},th,tl,  {1/0},0, 0,0 }
    //
    else if( cmd == 'D' ) {
#if 0
        uint8_t serverdown_on = msgbufp[1];
        uint16_t t = ((uint16_t)msgbufp[2] << 8) | msgbufp[3];
        uint8_t st = msgbuf[4];
        if( serverdown_on ) {
            serverdown_millis = t;
            serverdown_update_next = millis() + (t*10);
        } else {
            serverdown_millis = 0; // turn off serverdown mode
        }
        if( st == 0 ) {  // reset blink(1) state
            off();
        }
#endif
    }
    // version info
    else if( cmd == 'v' ) {
        msgbufp[2] = blink1_ver_major;
        msgbufp[3] = blink1_ver_minor;
    }
    else if( cmd == '!' ) { // testing testing
        msgbufp[0] = 0x55;
        msgbufp[1] = 0xAA;
        msgbufp[2] = 0; //usbHasBeenSetup;
    }
    else {

    }
}
