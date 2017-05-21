/**
  ******************************************************************************
  * @file    fontsepd.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-June-2014
  * @brief   Header for fonts files
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _FONTSEPD_H
#define _FONTSEPD_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#ifdef __AVR__
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#else
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define strlen_P strlen
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


typedef struct {
  const uint8_t *table;
  uint16_t Width;
  uint16_t Height;
} font_t;

extern font_t font14x20;
extern font_t font11x16;
extern font_t font7x12;
extern font_t font5x8;

#define GET_FONT()  pFont
   
#define LINE(x) ((x) * (((font_t *)GET_FONT())->Height))


#ifdef __cplusplus
}
#endif
  
#endif /* _FONTSEPD_H */
