/* Arduino Smart_Eink Library 
 * Copyright (C) 2016 by NOA Labs
 * Author  Bruce Guo (NOA Labs)
 *
 * This file is E-ink demo showing string.
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

#include <SPI.h>
#include <ssd1606.h>

#define E_CS   6  // CS ~ D6
#define E_DC   5  // D/C ~ D5
#define E_BSY  7  // BUSY ~ D7
#define E_RST  2  // RST ~ D2
#define E_BS   8  // BS ~ D8

/*
MOSI ~ D11
MISO ~ D12
CLK ~ D13
*/
EPD_SSD1606 Eink(E_CS, E_DC, E_BSY, E_RST);

void setup()
{
  // set BS to LOW for 4 line SPI (line can be attached to GND)
  pinMode(E_BS, OUTPUT);
  digitalWrite(E_BS, LOW);
  
  Eink.begin();
  Eink.fillScreen(COLOR_WHITE); // clear the screen

  Eink.setFont(&font5x8);
  Eink.setCursor(0, 0);
  Eink.println("Hello World!");
  Eink.println(1234.56);
  Eink.println(0xDEADBEEF, HEX);
  Eink.println();
  Eink.print(F("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada."));

  Eink.display(); 

  // BSY line acts not like in datasheet that
  // is why need delay in 2 sec before enter deep sleep
  delay(2000);
  Eink.sleep(true);
}

void loop(){}
