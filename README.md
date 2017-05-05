# Volume Knob with Blink1 led

This is an Arduino sketch to integrate a volume knob with a WS2812B led being controlled with the blink1 protocol.

The blink1 protocol expects a specific vid/pid and this is not currently
implemented here as it makes it harder to reprogram the device so changing the
blink1 utility is needed to make this work.
