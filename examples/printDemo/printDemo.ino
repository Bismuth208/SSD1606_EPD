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
/*
D/C ~ D5
CS ~ D6
BUSY ~ D7
BS ~ D8
MOSI ~ D11
MISO ~ D12
CLK ~ D13
RST ~ D2
*/


#include <SPI.h>
#include <ssd1606.h>

//EPD_SSD1606 Eink(int8_t _CS, int8_t _DC, int8_t _BSY, int8_t _RST);
EPD_SSD1606 Eink(6, 5, 7, 2);

void setup()
{
  //BS LOW for 4 line SPI
  pinMode(8,OUTPUT);
  digitalWrite(8, LOW);
  
  Eink.begin();
  Eink.setFont(&font5x8);
  Eink.fillScreen(COLOR_WHITE); // clear the screen

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
