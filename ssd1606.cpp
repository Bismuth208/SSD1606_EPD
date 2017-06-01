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
  - This is our library for the Adafruit ILI9341 Breakout and Shield
  - ----> http://www.adafruit.com/products/1651
  -
  - Check out the links above for our tutorials and wiring diagrams
  - These displays use SPI to communicate, 4 or 5 pins are required to
  - interface (RST is optional)
  - Adafruit invests time and resources providing this open source code,
  - please support Adafruit and open-source hardware by purchasing
  - products from Adafruit!
  -
  - Written by Limor Fried/Ladyada for Adafruit Industries.
  - MIT license, all text above must be included in any redistribution
  - ****************************************************/


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

#ifdef __AVR__
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#else
  #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
  #define strlen_P strlen
#endif

#include <SPI.h>
#include "ssd1606.h"

// -------------------------------------------------------------------------- //
// If the SPI library has transaction support, these functions
// establish settings and protect from interference from other
// libraries.  Otherwise, they simply do nothing.
#ifdef SPI_HAS_TRANSACTION
static inline void spi_begin(void) __attribute__((always_inline));
static inline void spi_begin(void) {
#if defined (ARDUINO_ARCH_ARC32)
  // max speed!
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
#else
  // max speed!
  SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
#endif
}
static inline void spi_end(void) __attribute__((always_inline));
static inline void spi_end(void) {
  SPI.endTransaction();
}
#else
#define spi_begin()
#define spi_end()
#endif

// -------------------------------------------------------------------------- //

static const uint8_t init_commands[] PROGMEM = {
  2, SSD1606_DPSLP, 0x00,         // exit deep sleep mode
  2, SSD1606_DEMDS, DE_YXII,      // data enter mode
  2, SSD1606_BFS, 0x1F,           // booster feedback used, in page 37
  2, SSD1606_WVCOMREG, 0xA0,      // vcom
  2, SSD1606_VBDSET, 0x63,        // select border waveform
  2, SSD1606_DUPCTRL1, 0x03,      // disable RAM bypass and set GS transition to GSA = GS0 and GSB = GS3
  3, SSD1606_RASTXSE, 0x00, 0x11, // RAM x address: start at 00h; end at 11h(17)->72 (0x48?)
  3, SSD1606_RASTYSE, 0x00, 0xAB, // RAM y address: start at 00h; start at ABh(171)->172
  2, SSD1606_RASTXAC, 0x00,       // set RAM x address count to 0;
  2, SSD1606_RASTYAC, 0x00,       // set RAM y address count to 0;
  
  91, SSD1606_WLUTREG,            // write data to LUT register
  0x00,0x00,0x00,0x55,0x00,0x00,0x55,0x55,0x00,
  0x55,0x55,0x55,0xAA,0xAA,0xAA,0xAA,0x15,0x15,
  0x15,0x15,0x05,0x05,0x05,0x05,0x01,0x01,0x01,
  0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x22,
  0xFB,0x22,0x1B,0x00,0x00,0x00,0x00,0x00,0x00,
  0
};

// -------------------------------------------------------------------------- //

// Constructor when using hardware SPI and reset connected to MCU reset.
// Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
EPD_SSD1606::EPD_SSD1606(int8_t cs, int8_t dc, int8_t bsy)
{
  _cs   = cs;
  _dc   = dc;
  _bsy  = bsy;
  _rst  = -1;
  hwSPI = true;
  
  cursor_x = cursor_y =0;
  wrap = true;
}

// Constructor when using hardware SPI and reset connected to some of pins.
// Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
EPD_SSD1606::EPD_SSD1606(int8_t cs, int8_t dc, int8_t bsy, int8_t rst)
{
  _cs   = cs;
  _dc   = dc;
  _bsy  = bsy;
  _rst  = rst;
  hwSPI = true;
  
  cursor_x = cursor_y =0;
  wrap = true;
}

// Constructor when using software SPI selected.
// reset connected to MCU reset
EPD_SSD1606::EPD_SSD1606(int8_t cs, int8_t dc, int8_t bsy,
                                      int8_t mosi, int8_t sclk, int8_t miso)
{
  _cs   = cs;
  _dc   = dc;
  _bsy  = bsy;
  _rst  = -1;
  _mosi = mosi;
  _miso = miso;
  _sclk = sclk;
  hwSPI = false;
  
  cursor_x = cursor_y =0;
  wrap = true;
}

// Constructor when using software SPI.  All output pins are configurable.
EPD_SSD1606::EPD_SSD1606(int8_t cs, int8_t dc, int8_t bsy, int8_t rst,
                                      int8_t mosi, int8_t sclk, int8_t miso)
{
  _cs   = cs;
  _dc   = dc;
  _bsy  = bsy;
  _rst  = rst;
  _mosi = mosi;
  _miso = miso;
  _sclk = sclk;
  hwSPI = false;
  
  cursor_x = cursor_y =0;
  wrap = true;
}
// -------------------------------------------------------------------------- //

/**
 * @brief  Init: GPIO, SPI, display e.t.c.
 * @param  None
 * @retval None
 */
void EPD_SSD1606::begin(void)
{
  if (_rst > 0) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, LOW);
  }
  
  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);
  
#if defined (USE_FAST_PINIO)
  csport    = portOutputRegister(digitalPinToPort(_cs));
  cspinmask = digitalPinToBitMask(_cs);
  dcport    = portOutputRegister(digitalPinToPort(_dc));
  dcpinmask = digitalPinToBitMask(_dc);
#endif
  
  if(hwSPI) { // Using hardware SPI
    SPI.begin();
    
#ifndef SPI_HAS_TRANSACTION
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
#if defined (_AVR__)
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
    mySPCR = SPCR;
#elif defined(TEENSYDUINO)
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
#elif defined (__arm__)
    SPI.setClockDivider(11); // 8-ish MHz (full! speed!)
#endif
#endif
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
    
#if defined (USE_FAST_PINIO)
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    clkpinmask  = digitalPinToBitMask(_sclk);
    mosiport    = portOutputRegister(digitalPinToPort(_mosi));
    mosipinmask = digitalPinToBitMask(_mosi);
    *clkport   &= ~clkpinmask;
    *mosiport  &= ~mosipinmask;
#endif
  }
  
  // toggle RST low to reset
  if (_rst > 0) {
    digitalWrite(_rst, HIGH);
  }
  
  commandList(init_commands);
  
  setFont(&EPD_DEFAULT_FONT);
}

/**
 * @brief  refresh the screen, need to be called by application every time the screen changed
 * @param  None
 * @retval None
 */
void EPD_SSD1606::display(void)
{
  // from mbed
  writeCommand(SSD1606_DUPCTRL2);
  writeData(0xC4);
  writeCommand(SSD1606_ADPUPDSC);
  
  /* Poll on the BUSY signal and wait for the EPD to be ready */
  while(digitalRead(_bsy) == LOW);
  closeChargePump();
}

/**
 * @brief  Disables the clock and the charge pump
 * @param  None
 * @retval None
 */
void EPD_SSD1606::closeChargePump(void)
{
  /* Write on the Display update control register */
  writeCommand(SSD1606_DUPCTRL2);
  
  /* Disable CP then Disable Clock signal */
  writeCommand(0x03);
  
  /* Launching the update: Nothing should interrupt this sequence in order
   to avoid display corruption */
  writeCommand(SSD1606_ADPUPDSC);
  while(digitalRead(_bsy) == LOW);
  //delay(2000); // BSY signal acts not like in datasheet! Thats is why delay is here..
}

/**
 * @brief  Clear whole screen
 * @param  color: selected color to be filed
 *         This parameter can be one of the following values:
 *            @arg  COLOR_WHITE
 *            @arg  COLOR_LIGHTGREY
 *            @arg  COLOR_DARKGREY
 *            @arg  COLOR_BLACK
 * @retval None
 */
void EPD_SSD1606::fillScreen(uint8_t color)
{
  setAddrWindow(0, 0, 171, 17);
  for(uint16_t i=0; i<SSD1606_SCREEN_SIZE; i++) {
    writeData(color);
  }
}

/**
 * @brief  Deep sleep mode.
 * @param  state: sleep or wakeup
 * @warning  From deep sleep only reset can wakeup display!
 * @retval None
 */
void EPD_SSD1606::sleep(bool state)
{
  if(state) {
    writeCommand(SSD1606_DPSLP); // deep sleep mode
    writeData(0x01);
  } else { // only reinit can wakeup display from deep sleep
    // toggle RST low to reset
    if (_rst > 0) {
      digitalWrite(_rst, LOW);
      digitalWrite(_rst, HIGH);
    }
    commandList(init_commands);
  }
}

/**
 * @brief  Send to display 4 pixels
 * @param  color: data and color whith 4 pixels
 * @retval None
 */
void EPD_SSD1606::drawPixel(uint8_t color)
{
  writeData(color);
}

/**
 * @brief  Draw pixel to display at position
 * @param  x: X position (0-171)
 * @param  y: Y position (0-71)
 * @param  color: data and color whith 1 pixel
 * @retval None
 */
void EPD_SSD1606::drawPixel(int16_t x, int16_t y, uint8_t color)
{
  uint8_t sector = y/4;
  uint8_t pixelPosition = y%4;
  
  setAddrWindow(x, sector, x, sector+1);
  
  switch(pixelPosition) {
    case 0: writeData(color | 0x3F); break;
    case 1: writeData(color | 0xCF); break;
    case 2: writeData(color | 0xF3); break;
    case 3: writeData(color | 0xFC); break;
  }
}

/**
 * @brief  Draw horizontal line
 * @param  x: X position where line start
 * @param  y: Y position where line start
 * @param  w: lenght of line
 * @retval None
 */
void EPD_SSD1606::drawFastHLine(int16_t x, int16_t y, int16_t w, uint8_t color)
{
  setAddrWindow(x, y, x+w, y);
  
  for(uint8_t index = 0; index < w; index++) {
    /* Prepare the register to write data on the RAM */
    writeData(color);
  }
}

/**
 * @brief  Draw vertical line
 * @param  x: X position where line start
 * @param  y: Y position where line start (0-17)
 * @param  h: lenght of line in sectors of 4 pixel each
 * @retval None
 */
void EPD_SSD1606::drawFastVLine(int16_t x, int16_t y, int16_t h, uint8_t color)
{
  setAddrWindow(x, y, x, y+h);
  
  for(uint8_t index = 0; index < h; index++) {
    /* Prepare the register to write data on the RAM */
    writeData(color);
  }
}

/**
 * @brief  Draw vertical line
 * @param  x: X position where line start
 * @param  y: Y position where line start
 * @param  h: lenght of line (0-71)
 * @retval None
 */
void EPD_SSD1606::drawVLine(int16_t x, int16_t y, int16_t h, uint8_t color)
{
  uint8_t sectors=0;
  uint8_t pixelsLeft=0;
  
  pixelsLeft = h%4;
  sectors = (h - pixelsLeft)>>2; // ... same as '/4' but sometimes it more effective
  if(pixelsLeft) { // yep, even single pixel consume whole sector :/
    setAddrWindow(x, y, x, y+sectors+1);
  } else {
    setAddrWindow(x, y, x, y+sectors);
  }
  
  for(uint8_t index = 0; index < sectors; index++) {
    /* Prepare the register to write data on the RAM */
    writeData(color);
  }
  
  // fill sector whith 1 to 3 pixels
  switch(pixelsLeft) {
    case 1: writeData(color | 0x3F); break;
    case 2: writeData(color | 0x0F); break;
    case 3: writeData(color | 0x03); break;
    default: break; // no pixels left
  }
}

/**
 * @brief  Draw non filled rectangle
 * @param  x: X position where line start
 * @param  y: Y position where line start
 * @param  w: wight
 * @param  h: height
 * @retval None
 */
void EPD_SSD1606::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color)
{
  /* Draw horizontal lines */
  drawFastHLine(x, y, w, color);
  drawFastHLine(x, y + h, w + 1, color);
  
  /* Draw vertical lines */
  drawFastVLine(x, y, h, color);
  drawFastVLine(x + w, y , h, color);
}

/**
 * @brief  Draw filled rectangle
 * @param  x: X position where line start
 * @param  y: Y position where line start
 * @param  w: wight
 * @param  h: height
 * @retval None
 */
void EPD_SSD1606::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color)
{
  /* Set the rectangle */
  setAddrWindow(x, y, x + w, y + h);
  
  for(int16_t index = 0; index < w*(h*4); index++) {
    writeData(color);
  }
}

/**
 * @brief  Set dispay bit depth
 * @param  depth: mode 2 bit or 1 bit per pixel
 * @retval None
 */
void EPD_SSD1606::setBitDepth(uint8_t depth)
{
  writeCommand(SSD1606_DPCTRL);
  writeData(depth);
}

// ----------------------------------------------------------------------------//
// ----------------------------------------------------------------------------//

/**
 * @brief  Set addr window where data in RAM display will be placed
 * @param  x0: X start  address   0~17
 * @param  y0: Y start  address   0~171
 * @param  x1: X end    address   0~17
 * @param  y1: Y end    address   0~171
 * @retval None
 */
void EPD_SSD1606::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  writeCommand(SSD1606_RASTXSE); // set RAM x address start/end
  writeData(y0);
  writeData(y1);
  writeCommand(SSD1606_RASTYSE); // set RAM y address start/end
  writeData(x0);
  writeData(x1);
  writeCommand(SSD1606_RASTXAC); // set RAM x address count to x0;
  writeData(y0);
  writeCommand(SSD1606_RASTYAC); // set RAM y address count to y0;
  writeData(x0);
  
  writeCommand(SSD1606_RAMWR);
}

// ----------------------------------------------------------------------------//
// ----------------------------------------------------------------------------//
#if ARDUINO >= 100
size_t EPD_SSD1606::write(uint8_t c) {
#else
  void EPD_SSD1606::write(uint8_t c) {
#endif
    
#if 0
    if(c == '\n') {
      cursor_y += textsize*8;
      cursor_x  = 0;
    } else if(c == '\r') {
      // skip em
    } else {
      if(wrap && ((cursor_x + textsize * 6) >= _width)) { // Heading off edge?
        cursor_x  = 0;            // Reset x to zero
        cursor_y += textsize * 8; // Advance y one line
      }
      drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
      cursor_x += textsize * 6;
    }
#endif
    
#if 1
    if(c == '\n') {
      cursor_y += pFont->Height;
      cursor_x  = 0;
    } else if(c == '\r') {
      // skip em
    } else {
      if(wrap && ((cursor_x + pFont->Width) >= SSD1606_WIDTH)) { // Heading off edge?
        cursor_x  = 0;            // Reset x to zero
        cursor_y += pFont->Height; // Advance y one line
      }
      displayChar(cursor_x, cursor_y, c);
      cursor_x += pFont->Width;
    }
#endif
    
#if ARDUINO >= 100
    return 1;
#endif
}
  
void EPD_SSD1606::setTextWrap(bool w)
{
  wrap = w;
}
  
void EPD_SSD1606::setCursor(int16_t x, int16_t y)
{
  cursor_x = x;
  cursor_y = y;
}
  
/**
 * @brief  Sets the Text Font.
 * @param  pFonts: specifies the layer font to be used.
 * @retval None
 */
void EPD_SSD1606::setFont(font_t *pFonts)
{
  pFont = pFonts;
}

/**
 * @brief  Displays one character.
 * @param  Xpos: start column address.
 * @param  Ypos: the Line where to display the character shape.
 * @param  Ascii: character ascii code, must be between 0x20 and 0x7E.
 * @retval None
 */
void EPD_SSD1606::displayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii)
{
  Ascii -= 32;
  
  drawChar(Xpos, Ypos, &pFont->table[Ascii * ((pFont->Height) * (pFont->Width))]);
}

/**
 * @brief  Draws a character on EPD.
 * @param  Xpos: specifies the X position, can be a value from 0 to 171
 * @param  Ypos: specifies the Y position, can be a value from 0 to 17
 * @param  c: pointer to the character data
 * @retval None
 */
void EPD_SSD1606::drawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c)
{
  uint16_t height = pFont->Height;
  uint16_t width = pFont->Width;
  uint32_t data_length = (height * width);
  
  /* Set the Character display window */
  setAddrWindow(Xpos, Ypos, (Xpos + width - 1), (Ypos + height - 1));
  
  for(uint32_t index = 0; index < data_length; index++) {
    writeData(pgm_read_byte(c + index));
  }
}

/**
 * @brief  Displays characters on the EPD.
 * @param  Xpos: X position
 * @param  Ypos: Y position
 * @param  text: Pointer to string to display on EPD
 * @param  mode: Display mode
 *          This parameter can be one of the following values:
 *            @arg  ALIGN_CENTER
 *            @arg  ALIGN_RIGHT
 *            @arg  ALIGN_LEFT
 * @retval None
 */
void EPD_SSD1606::displayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *text, textAlign_t mode)
{
  uint16_t refcolumn = 1, i = 0;
  uint32_t size = 0, xsize = 0;
  uint8_t  *ptr = text;
  
  /* Get the text size */
  while (*ptr++) size ++ ;
  
  /* Characters number per line */
  xsize = (width()/pFont->Width);
  
  switch (mode)
  {
    case ALIGN_CENTER:
    {
      refcolumn = Xpos + ((xsize - size)* pFont->Width) / 2;
      break;
    }
    case ALIGN_LEFT:
    {
      refcolumn = Xpos;
      break;
    }
    case ALIGN_RIGHT:
    {
      refcolumn =  - Xpos + ((xsize - size)*pFont->Width);
      break;
    }
    default:
    {
      refcolumn = Xpos;
      break;
    }
  }
  
  /* Send the string character by character on EPD */
  while ((*text != 0) & (((width() - (i*pFont->Width)) & 0xFFFF) >= pFont->Width))
  {
    /* Display one character on EPD */
    displayChar(refcolumn, Ypos, *text);
    /* Decrement the column position by 16 */
    refcolumn += pFont->Width;
    /* Point on the next character */
    text++;
    i++;
  }
}

/**
 * @brief  Displays a character on the EPD.
 * @param  Line: Line where to display the character shape
 *          This parameter can be one of the following values:
 *            @arg  0..8: if the Current fonts is font5x8
 *            @arg  0..5: if the Current fonts is font7x12
 *            @arg  0..3: if the Current fonts is font11x16
 *            @arg  0..2: if the Current fonts is font14x20
 * @param  ptr: Pointer to string to display on EPD
 * @retval None
 */
void EPD_SSD1606::displayStringAtLine(uint16_t Line, uint8_t *ptr)
{
  displayStringAt(0, LINE(Line), ptr, ALIGN_LEFT);
}

// ----------------------------------------------------------------------------//
// ----------------------------------------------------------------------------//
/**
 * @brief  Displays picture..
 * @param  pData: picture address.
 * @param  Xpos:  Image X position in the EPD
 * @param  Ypos:  Image Y position in the EPD
 * @param  Xsize: Image X size in the EPD
 * @note   Xsize have to be a multiple of 4
 * @param  Ysize: Image Y size in the EPD
 * @retval None
 */
void EPD_SSD1606::drawImageInt(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pData)
{
  uint32_t i, j = 0;
  uint8_t pixels_4 = 0;
  uint8_t pixels_4_grey[4];
  uint8_t nb_4_pixels, data_res = 0;
  
  /* X size is a multiple of 8 */
  if ((Xsize % 8) == 0)
  {
    for (i= 0; i< ((((Ysize) * (Xsize/4)))/2) ; i++)
    {
      /* Get the current data */
      pixels_4 = pData[i];
      if (pixels_4 !=0)
      {
        /* One byte read codes 8 pixels in 1-bit bitmap */
        for (nb_4_pixels = 0; nb_4_pixels < 2; nb_4_pixels++)
        {
          /* Processing 8 pixels */
          /* Preparing the 4 pixels coded with 4 grey level per pixel
           from a monochrome xbm file */
          for (j= 0; j<4; j++)
          {
            if (((pixels_4) & 0x01) == 1)
            {
              /* Two LSB is coding black in 4 grey level */
              pixels_4_grey[j] &= 0xFC;
            }
            else
            {
              /* Two LSB is coded white in 4 grey level */
              pixels_4_grey[j] |= 0x03;
            }
            pixels_4 = pixels_4 >> 1;
          }
          
          /* Processing 4 pixels */
          /* Format the data to have the Lower pixel number sent on the MSB for the SPI to fit with the RAM
           EPD topology */
          data_res = pixels_4_grey[0] << 6 | pixels_4_grey[1] << 4 | pixels_4_grey[2] << 2 | pixels_4_grey[3] << 0;
          
          /* Send the data to the EPD's RAM through SPI */
          writeData(data_res);
        }
      }
      else
      {
        /* 1 byte read from xbm files is equivalent to 8 pixels in the
         other words 2 bytes to be transferred */
        writeData(0xFF);
        writeData(0xFF);
      }
    }
  }
  
  /* X size is a multiple of 4 */
  else
  {
    for (i= 0; i< ((((Ysize) * ((Xsize/4)+1))/2)) ; i++)
    {
      /* Get the current data */
      pixels_4 = pData[i];
      if (((i+1) % (((Xsize/4)+1)/2)) != 0)
      {
        if (pixels_4 !=0)
        {
          /* One byte read codes 8 pixels in 1-bit bitmap */
          for (nb_4_pixels = 0; nb_4_pixels < 2; nb_4_pixels++)
          {
            /* Processing 8 pixels */
            /* Preparing the 4 pixels coded with 4 grey level per pixel
             from a monochrome xbm file */
            for (j= 0; j<4; j++)
            {
              if (((pixels_4) & 0x01) == 1)
              {
                /* Two LSB is coding black in 4 grey level */
                pixels_4_grey[j] &= 0xFC;
              }
              else
              {
                /* Two LSB is coded white in 4 grey level */
                pixels_4_grey[j] |= 0x03;
              }
              pixels_4 = pixels_4 >> 1;
            }
            
            /* Processing 4 pixels */
            /* Format the data to have the Lower pixel number sent on the MSB for the SPI to fit with the RAM
             EPD topology */
            data_res = pixels_4_grey[0] << 6 | pixels_4_grey[1] << 4 | pixels_4_grey[2] << 2 | pixels_4_grey[3] << 0;
            
            /* Send the data to the EPD's RAM through SPI */
            writeData(data_res);
          }
        }
        else if (pixels_4 == 0)
        {
          /* One byte read from xbm files is equivalent to 8 pixels in the
           other words Two bytes to be transferred */
          writeData(0xFF);
          writeData(0xFF);
        }
      }
      
      else if (((i+1) % (((Xsize/4)+1)/2)) == 0)
      {
        if (pixels_4 !=0xf0)
        {
          /* Processing 8 pixels */
          /* Preparing the 4 pixels coded with 4 grey level per pixel
           from a monochrome xbm file */
          for (j= 0; j<4; j++)
          {
            if (((pixels_4) & 0x01) == 1)
            {
              /* 2 LSB is coding black in 4 grey level */
              pixels_4_grey[j] &= 0xFC;
            }
            else
            {
              /* 2 LSB is coded white in 4 grey level */
              pixels_4_grey[j] |= 0x03;
            }
            pixels_4 = pixels_4 >> 1;
          }
          
          /* Processing 4 pixels */
          /* Format the data to have the Lower pixel number sent on the MSB for the SPI to fit with the RAM
           EPD topology */
          data_res = pixels_4_grey[0] << 6 | pixels_4_grey[1] << 4 | pixels_4_grey[2] << 2 | pixels_4_grey[3] << 0;
          
          /* Send the data to the EPD's RAM through SPI */
          writeData(data_res);
        }
        else if (pixels_4 == 0xf0)
        {
          /* One byte to be transferred */
          writeData(0xFF);
        }
      }
    }
  }
}

/**
 * @brief  Draws an Image.
 * @param  Xpos: X position in the EPD
 * @param  Ypos: Y position in the EPD
 * @param  Xsize: X size in the EPD
 * @param  Ysize: Y size in the EPD
 * @param  pData: Pointer to the Image address
 * @retval None
 */
void EPD_SSD1606::drawImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pData)
{
  /* Set display window */
  setAddrWindow(Xpos, Ypos, (Xpos+Ysize-1), (Ypos+(Xsize/4)-1));
  
  drawImageInt(Xpos, Ypos, Xsize, Ysize, pData);
}

// ----------------------------------------------------------------------------//
// ----------------------------------------------------------------------------//

/**
 * @brief  Low level driver, send data to diplay
 * @param  c: data to send
 * @retval None
 */
void EPD_SSD1606::spiwrite(uint8_t c)
{
  if (hwSPI) {
#if defined (__AVR__)
#ifndef SPI_HAS_TRANSACTION
    uint8_t backupSPCR = SPCR;
    SPCR = mySPCR;
#endif
    SPDR = c;
    while(!(SPSR & _BV(SPIF)));
#ifndef SPI_HAS_TRANSACTION
    SPCR = backupSPCR;
#endif
#else
    SPI.transfer(c);
#endif
  } else {
#if defined(ESP8266) || defined (ARDUINO_ARCH_ARC32)
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
        digitalWrite(_mosi, HIGH);
      } else {
        digitalWrite(_mosi, LOW);
      }
      digitalWrite(_sclk, HIGH);
      digitalWrite(_sclk, LOW);
    }
#else
    // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
        //digitalWrite(_mosi, HIGH);
        *mosiport |=  mosipinmask;
      } else {
        //digitalWrite(_mosi, LOW);
        *mosiport &= ~mosipinmask;
      }
      //digitalWrite(_sclk, HIGH);
      *clkport |=  clkpinmask;
      //digitalWrite(_sclk, LOW);
      *clkport &= ~clkpinmask;
    }
#endif
  }
}

/**
 * @brief  Send command to diplay throw SPI
 * @param  c: data to send
 * @retval None
 */
void EPD_SSD1606::writeCommand(uint8_t c)
{
#if defined (USE_FAST_PINIO)
  *dcport &= ~dcpinmask;
  *csport &= ~cspinmask;
#else
  digitalWrite(_dc, LOW);
  digitalWrite(_sclk, LOW);
  digitalWrite(_cs, LOW);
#endif
  
  spiwrite(c);
  
#if defined (USE_FAST_PINIO)
  *csport |= cspinmask;
#else
  digitalWrite(_cs, HIGH);
#endif
}

/**
 * @brief  Send data to diplay throw SPI
 * @param  c: data to send
 * @retval None
 */
void EPD_SSD1606::writeData(uint8_t c)
{
#if defined (USE_FAST_PINIO)
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
#else
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
#endif
  
  spiwrite(c);
  
#if defined (USE_FAST_PINIO)
  *csport |= cspinmask;
#else
  digitalWrite(_cs, HIGH);
#endif
}

/**
 * @brief  Send to display init sequence
 * @param  addr: pointer to array whith init sequence
 * @retval None
 */
void EPD_SSD1606::commandList(const uint8_t *addr)
{
  int8_t count;
  while (1) {
    count = pgm_read_byte(addr++);        // Number of commands to follow
    if (count-- == 0) break;              // For each command...
    writeCommand(pgm_read_byte(addr++));  // Read, issue command
    while (count--) {                     // For each argument...
      writeData(pgm_read_byte(addr++));   // Read, issue argument
    }
  }
}

