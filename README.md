# Snake for ESP32 with Rust

This project implements the game Snake for an ESP32 microcontroller,
written in Rust.

This was written as a basic starter project.

Hardware:
- [ESP32-WROOM-32 board](https://www.az-delivery.de/en/products/esp32-developmentboard)
- [KY-023 Joystick module](https://www.az-delivery.de/en/products/joystick-modul) - note this has a click input too.
- [0.96 inch SSD1306 128x64 I2C OLED display](https://www.az-delivery.de/en/products/0-96zolldisplay)

Photo:

![Photo of breadboard](./breadboardphoto_small.jpg)


Embassy is used to run concurrent tasks (checking for input at a faster
rate than the game update rate). 

The joystick URX and URY axes inputs are handled with a built-in
Analogue-Digital Converter (be careful about the choice of GPIO pins as
the selection is limited). And the button click is handled with an
interrupt.

## Notes

Note the input voltage to the joystick had to be lowered via a voltage
divider to get a usable range, the voltage divider here used one 1k and
one 2k Ohm resistors to the 5V input.

Note the SCL and SDA pins to the I2C display needed pull-up resistors,
here I used 2k Ohm resistors to 5V.

## Resources

- ESP32 GPIO Interrupts - https://blog.theembeddedrustacean.com/esp32-embedded-rust-at-the-hal-gpio-interrupts
- GPIO usage with Embassy on ESP32 - https://dev.to/theembeddedrustacean/embassy-on-esp-gpio-5594
- I2C Scanner - https://dev.to/theembeddedrustacean/esp32-embedded-rust-at-the-hal-i2c-scanner-54mg

