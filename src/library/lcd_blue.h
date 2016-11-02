#ifndef __LCD_BLUE_H
#define __LCD_BLUE_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "delay.h"
#include "lcd_font.h"
#include "uart.h"

#include <stdio.h>
#include <stdarg.h>

#define TFT_SPI	SPI1
#define TFT_CS_PIN		GPIO_Pin_4
#define TFT_DC_PIN		GPIO_Pin_3
#define TFT_RST_PIN		GPIO_Pin_2
#define TFT_CS_PORT		GPIOA
#define TFT_DC_PORT		GPIOA
#define TFT_RST_PORT	GPIOA

#define X_MAX_PIXEL	        128
#define Y_MAX_PIXEL	        160

#define RED  		0xf800
#define GREEN		0x07e0
#define BLUE 		0x001f
#define WHITE		0xffff
#define BLACK		0x0000
#define YELLOW  0xFFE0
#define GRAY0   0xEF7D
#define GRAY1   0x8410
#define GRAY2   0x4208

#define	RGB888TO565(RGB888)  (((RGB888 >> 8) & 0xF800) |((RGB888 >> 5) & 0x07E0) | ((RGB888 >> 3) & 0x001F))
void tft_spi_init(void);
void tft_write_command(u8 command);
void tft_write_data(u8 data);
void tft_reset(void);
void tft_init(u8 dir, u16 bg_color, u16 text_color);
void tft_set_bg_color(u16 color);
void tft_set_text_color(u16 color);
void tft_set_font(const FONT_INFO *font);
void tft_setregion(u16 x1, u16 x2, u16 y1, u16 y2);
void tft_put_pixel(u8 x, u8 y, u16 color);
void tft_clear(void);
void tft_prints(u8 x, u8 y, const char * pstr, ...);
void tft_fill_area(u8 x1, u8 y1, u8 x2, u8 y2, u16 color);
void tft_print_image(u8 x, u8 y, tImage image);

#endif
