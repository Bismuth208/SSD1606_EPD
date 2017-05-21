# SSD1606
*Graphics library for SSD1606 EPD driver for Arduino*
***

Library for Eink (EPD) based on SSD1606 driver controller.
Resolution: 172x72;
Actual resolution: 172x18;

As Y ordinate have organisation in bytes, then 18 = 72/4;
4 - mean 4 pixels per byte (2 bits per pixel);

At current moment no framebuffer avaliable!
That mean what draw graphics primitives are almost imposible...

Arduino's' *.print* fully supported!
Deep sleep mode also avaliable!

EPD connection: SPI;
Lines: MOSI, SCK, CS, D/C, RES, BSY, VCC, GND.

[To see how does it work check this link](https://www.youtube.com/channel/UCDXVQ9ZfQl8Ddeu_3qiwSiA "My YouTube channel")
***

  
* As *MCU* can be used:
  * Any capable board with *Arduino IDE*: Uno, Mega, nano, STM32, e.t.c.

> ### ATTENTION!
>  * This project is still unstable and in develop!
>  * Any changes are possible at any time,
>  * and no backward capability is guaranteed!
  
