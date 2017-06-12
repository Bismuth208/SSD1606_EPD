/* Arduino Smart_Eink Library
 * Copyright (C) 2016 by NOA Labs
 * Author  Bruce Guo (NOA Labs)
 *
 * This file is work for Eink.
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this Library. If not, see
 * <http://www.gnu.org/licenses/>.
 */

/***************************************************
 This is our library for the Adafruit ILI9341 Breakout and Shield
 ----> http://www.adafruit.com/products/1651
 
 Check out the links above for our tutorials and wiring diagrams
 These displays use SPI to communicate, 4 or 5 pins are required to
 interface (RST is optional)
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!
 
 Written by Limor Fried/Ladyada for Adafruit Industries.
 MIT license, all text above must be included in any redistribution
 ****************************************************/


/*
 * Created by: Antonov Alexandr (Bismuth208)
 * e-mail: bismuth20883@gmail.com
 *
 * I Merged libraries: NOA Labs + STM32Discovery + Adafruit GFX
 *
 * Library for EPD (Eink) module based on SSD1606 driver
 * resolution: 172x72
 * bit-depth: 1bit or 2bit
 *
 * MCU: almost all supported by Arduino IDE
 *
 * Its difficult to select copyright...
 * that is why i left them both!
 *
 */

// -------------------------------------------------------------------------- //

#include <stdint.h>

#ifndef _SSD1606_H
#define _SSD1606_H

#if ARDUINO >= 100
  #include "Arduino.h"
  #include "Print.h"
#else
  #include "WProgram.h"
#endif

#include "fontsepd.h"

#if defined (__AVR__) || defined(TEENSYDUINO) || defined (__arm__)
#define USE_FAST_PINIO
#endif


#if defined (USE_FAST_PINIO)
  #define SET_CS_LOW  *csport &= ~cspinmask;
  #define SET_CS_HI   *csport |= cspinmask;
  #define SET_DC_HI   *dcport |=  dcpinmask;
  #define SET_DC_LOW  *dcport &= ~dcpinmask;
#else
  #define SET_CS_LOW  digitalWrite(_cs, LOW);
  #define SET_CS_HI   digitalWrite(_cs, HIGH);
  #define SET_DC_HI   digitalWrite(_dc, HIGH);
  #define SET_DC_LOW  digitalWrite(_dc, LOW);
#endif

#define ENABLE_CMD    SET_DC_LOW SET_CS_LOW
#define ENABLE_DATA   SET_DC_HI  SET_CS_LOW
#define DISABLE_DATA  SET_CS_HI

#define SSD1606_WIDTH  172
#define SSD1606_HEIGHT 18

#define SSD1606_1_BIT_PP   1   // bits per pixel
#define SSD1606_2_BIT_PP   2   // bits per pixel

#define BYTE_SIZE          8

#define SSD1606_SCREEN_SIZE  ((SSD1606_WIDTH*SSD1606_HEIGHT*4*SSD1606_2_BIT_PP)/BYTE_SIZE) // == 3096


// for setBitDepth() func
#define SSD1606_1_BIT_MODE   0x01
#define SSD1606_2_BIT_MODE   0x00


// -------------------------------------------------------------------------- //
#define SSD1606_DOCTRL    0x01   // Driver output control
#define SSD1606_GDVCTRL   0x03   // Gate Driving voltage Control
#define SSD1606_SDVCTRL   0x04   // Source Driving voltage Control
#define SSD1606_DPCTRL    0x07   // Display Control ( select bit depth 1bit or 2 bit )
#define SSD1606_GSOVPC    0x0B   // Gate and Source non overlap period Control
#define SSD1606_GSSTPOS   0x0F   // Gate scan start position
#define SSD1606_DPSLP     0x10   // deep sleep control
#define SSD1606_DEMDS     0x11   // Data Entry mode setting
#define SSD1606_SWRESET   0x12
// ------ Temperature Sensor Control ------ //
#define SSD1606_WRRTEMPSC 0x1A   // Write to temperature register
#define SSD1606_WRRTEMPSC 0x1B   // Read from temperature register
#define SSD1606_WRCTEMPSC 0x1C   // Write Command to temperature sensor
#define SSD1606_WRCTEMPSC 0x1D   // Load temperature register with temperature sensor reading
// ---------------------------------------- //
#define SSD1606_ADPUPDSC  0x20   // Master Activation; Activate Display Update Sequence
#define SSD1606_DUPCTRL1  0x21   // Display Update Control 1
#define SSD1606_DUPCTRL2  0x22   // Display Update Sequence; Control 2
#define SSD1606_RAMWR     0x24   // Write RAM
#define SSD1606_RAMRD     0x25   // Read RAM
#define SSD1606_VCOMS     0x28   // VCOM Sense
#define SSD1606_VCOMSD    0x29   // VCOM Sense duration
#define SSD1606_PVCOMOTP  0x2A   // Program VCOM OTP
#define SSD1606_WVCOMREG  0x2C   // Write VCOM registe
#define SSD1606_ROTPREG   0x2D   // Read OTP registers
#define SSD1606_PWSOTP    0x30   // Program WS OTP
#define SSD1606_WLUTREG   0x32   // Write LUT register
#define SSD1606_RLUTREG   0x33   // Read LUT register
#define SSD1606_POTPSEL   0x36   // Program OTP selection
#define SSD1606_OTPSELC   0x37   // OTP selection control
#define SSD1606_DLPSET    0x3A   // Set dummy line period
#define SSD1606_GLWSET    0x3B   // Set Gate line width
#define SSD1606_VBDSET    0x3C   // Select border waveform for VBD
#define SSD1606_RASTXSE   0x44   // Set RAM X - address Start / End position
#define SSD1606_RASTYSE   0x45   // Set RAM Y - address Start / End position
#define SSD1606_RASTXAC   0x4E   // Set RAM X address counter
#define SSD1606_RASTYAC   0x4F   // Set RAM Y address counter
#define SSD1606_BFS       0xF0   // Booster Feedback Selection
#define SSD1606_NOP       0xFF
// -------------------------------------------------------------------------- //

// adress automatic increment
#define DE_YXDD     0x00  // Y decrement, X decrement
#define DE_YXDI     0x01  // Y decrement, X increment
#define DE_YXID     0x02  // Y increment, X decrement
#define DE_YXII     0x03  // Y increment, X increment; POR state

#define DE_ACX      0x00  // Addr counter is updated in the X direction; POR state
#define DE_ACY      0x04  // Addr counter is updated in the Y direction;

// -------------------------------------------------------------------------- //


// Color definitions
#define COLOR_WHITE       0xFF
#define COLOR_LIGHTGREY   0xAA
#define COLOR_DARKGREY    0x55
#define COLOR_BLACK       0x00

//#define COLOR_WHITE       0x11
//#define COLOR_LIGHTGREY   0x10
//#define COLOR_DARKGREY    0x01
//#define COLOR_BLACK       0x00


#define EPD_DEFAULT_FONT         font5x8


/**
 * @brief  Line mode structures definition
 */
typedef enum {
  ALIGN_CENTER             = 0x01,    /*!< Center mode */
  ALIGN_RIGHT              = 0x02,    /*!< Right mode  */
  ALIGN_LEFT               = 0x03     /*!< Left mode   */
} textAlign_t;


class EPD_SSD1606 : public Print {
  
public:
  EPD_SSD1606(int8_t cs, int8_t dc, int8_t bsy, int8_t rst = -1);
  EPD_SSD1606(int8_t cs, int8_t dc, int8_t bsy, int8_t mosi, int8_t sclk,
                                                int8_t miso, int8_t rst = -1);
  
  void begin(void);
  
  void fillScreen(uint8_t color);
  void display(void);
  void setBitDepth(uint8_t depth);
  void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
  void sleep(bool state);
  
  
  void drawPixel(uint8_t color);
  void drawPixel(int16_t x, int16_t y, uint8_t color);
  void drawVLine(int16_t x, int16_t y, int16_t h, uint8_t color);
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint8_t color);
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint8_t color);
  void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);
  
  void drawImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pData);
  void drawChar(uint16_t Xpos, uint16_t Ypos, uint8_t c);
  void displayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *text, textAlign_t align);
  void displayStringAtLine(uint16_t Line, uint8_t *ptr);
  
  void setFont(font_t *pFonts);
  void setTextWrap(bool w);
  void setCursor(int16_t x, int16_t y);
  
#if ARDUINO >= 100
  virtual size_t write(uint8_t);
#else
  virtual void   write(uint8_t);
#endif
  
  uint16_t width() { return SSD1606_WIDTH;}
  uint16_t height() {return SSD1606_HEIGHT;}
  
private:
  
  int16_t cursor_x, cursor_y;
  bool wrap;
  font_t *pFont;
  
  void drawImageInt(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pData);
  
  void closeChargePump(void);
  
  void spiwrite(uint8_t);
  void writeCommand(uint8_t c);
  void writeData(uint8_t c);
  
  void commandList(const uint8_t *addr);
  
  
  bool hwSPI;
#if defined (__AVR__) || defined(TEENSYDUINO)
  uint8_t mySPCR;
  volatile uint8_t *mosiport, *clkport, *dcport, *rsport, *csport, *bsyport;
  int8_t  _cs, _dc, _rst, _mosi, _miso, _sclk, _bsy;
  uint8_t  mosipinmask, clkpinmask, cspinmask, dcpinmask, bsymask;
#elif defined (__arm__)
  volatile RwReg *mosiport, *clkport, *dcport, *rsport, *csport, *bsyport;
  int32_t  _cs, _dc, _rst, _mosi, _miso, _sclk, _bsy;
  uint32_t  mosipinmask, clkpinmask, cspinmask, dcpinmask, bsymask;
#elif defined (ESP8266)
  int32_t  _cs, _dc, _rst, _mosi, _miso, _sclk, _bsy;
#else
  int8_t  _cs, _dc, _rst, _mosi, _miso, _sclk, _bsy;
#endif
};

#endif /* _SSD1606_H */
