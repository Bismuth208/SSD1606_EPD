/*
 * This is the example of GUI for simple
 * meteostation based on Eink display
 *
 * Author: Antonov Alexander (Bismuth208)
 */


#include <SPI.h>
#include <ssd1606.h>

#define TIME_X_POS   0
#define TIME_Y_POS   12

#define DATE_X_POS   2
#define DATE_Y_POS   9

#define WEECK_X_POS   70
#define WEECK_Y_POS   14

#define BATT_X_POS   70
#define BATT_Y_POS   12

#define TEMP_X_POS   110
#define TEMP_Y_POS   14

#define PRESURE_X_POS   110
#define PRESURE_Y_POS   12

#define HUMIDITY_X_POS  110
#define HUMIDITY_Y_POS  10

// ==== for presure history in graph ==== //
#define MAX_MESURES               170
#define BAR_GRAPH_X_POS           0
#define BAR_GRAPH_Y_POS           0
#define PRESURE_GRAPH_MIN         30      // vertical line graph for every N minutes
// ====================================== //


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

// --------------------------------------------------------- //

void setup()
{
  // set BS to LOW for 4 line SPI (line can be attached to GND)
  pinMode(E_BS, OUTPUT);
  digitalWrite(E_BS, LOW);

  Eink.begin();
  drawMeteoScreen();
  clearBarHistory();
  Eink.display();

  // BSY line acts not like in datasheet that
  // is why need delay in 2 sec before enter deep sleep
  delay(2000);
  Eink.sleep(true);
}

void loop() {}


void drawMeteoScreen(void)
{
  Eink.fillScreen(COLOR_WHITE);

  Eink.setFont(&font14x20);
  Eink.setCursor(TIME_X_POS, TIME_Y_POS);
  Eink.print(F("20:40"));

  Eink.setFont(&font7x12);
  Eink.setCursor(WEECK_X_POS, WEECK_Y_POS);
  Eink.print(F("Thu"));

  Eink.setCursor(DATE_X_POS, DATE_Y_POS);
  Eink.print(F("01/06/17"));

  Eink.setFont(&font5x8);
  Eink.setCursor(BATT_X_POS, BATT_Y_POS);
  Eink.print(F("4.65V"));

  Eink.setCursor(TEMP_X_POS, TEMP_Y_POS);
  Eink.print(F("25.30 C"));

  Eink.setCursor(PRESURE_X_POS, PRESURE_Y_POS);
  Eink.print(F("741.09 mm"));

  Eink.setCursor(HUMIDITY_X_POS, HUMIDITY_Y_POS);
  Eink.print(F("81.0 %")); 
}

void clearBarHistory(void)
{
  Eink.fillRect(BAR_GRAPH_X_POS, BAR_GRAPH_Y_POS, MAX_MESURES, 8, COLOR_WHITE);

  // draw lines every PRESURE_GRAPH_MIN minutes
  for(uint8_t i=1; i <= (MAX_MESURES/PRESURE_GRAPH_MIN); i++) {
    Eink.drawVLine(BAR_GRAPH_X_POS+i*PRESURE_GRAPH_MIN, BAR_GRAPH_Y_POS, 36, COLOR_DARKGREY);
  }
}
