#include "lcd_blue.h"

u8 curr_dir = 0;
u16 curr_bg_color = BLACK;
u16 curr_text_color = BLACK;
const FONT_INFO *curr_font = &microsoftSansSerif_8ptFontInfo;

void tft_change_dir(u8 *x, u8 *y, u8 dir) {
	u8 temp = 0;
	switch (dir) {
		case 0:
			break;
		case 1:
			temp = *y;
			*y = *x;
			*x = X_MAX_PIXEL - temp - 1;
			break;
		case 2:
			*y = Y_MAX_PIXEL - *y - 1;
			*x = X_MAX_PIXEL - *x - 1;
			break;
		case 3:
			temp = *y;
			*y = Y_MAX_PIXEL - *x - 1;
			*x = temp;
			break;
	}
}

void tft_spi_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	/* Enable GPIOA for RST pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable GPIOF for DC Pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure TFT_SPI Pin: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure TFT_SPI Pin: CS */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	 
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
	SPI_Init(TFT_SPI, &SPI_InitStructure);
	
	SPI_Cmd(TFT_SPI, ENABLE);
	SPI_CalculateCRC(TFT_SPI, DISABLE);
	SPI_SSOutputCmd(TFT_SPI, DISABLE);
}

void tft_write_command(u8 command) {
	GPIO_ResetBits(TFT_CS_PORT, TFT_CS_PIN);
	GPIO_ResetBits(TFT_DC_PORT, TFT_DC_PIN);
	while (SPI_I2S_GetFlagStatus(TFT_SPI, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(TFT_SPI, command);
	while (SPI_I2S_GetFlagStatus(TFT_SPI, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_I2S_ReceiveData(TFT_SPI);
	GPIO_SetBits(TFT_CS_PORT, TFT_CS_PIN);
}

void tft_write_data(u8 data) {
	GPIO_ResetBits(TFT_CS_PORT, TFT_CS_PIN);
	GPIO_SetBits(TFT_DC_PORT, TFT_DC_PIN);
	while (SPI_I2S_GetFlagStatus(TFT_SPI, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(TFT_SPI, data);
	while (SPI_I2S_GetFlagStatus(TFT_SPI, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_I2S_ReceiveData(TFT_SPI);
	GPIO_SetBits(TFT_CS_PORT, TFT_CS_PIN);
}

void tft_reset(void) {
	GPIO_ResetBits(TFT_RST_PORT, TFT_RST_PIN);
	_delay_ms(100);
	GPIO_SetBits(TFT_RST_PORT, TFT_RST_PIN);
	_delay_ms(50);
}

void tft_init(u8 dir, u16 bg_color, u16 text_color) {	
	tft_spi_init();
	tft_reset();
	
	//LCD Init For 1.44Inch LCD Panel with ST7735R.
	tft_write_command(0x11);//Sleep exit 
	_delay_ms(120);
		
	//ST7735R Frame Rate
	tft_write_command(0xB1); 
	tft_write_data(0x01); 
	tft_write_data(0x2C); 
	tft_write_data(0x2D); 

	tft_write_command(0xB2); 
	tft_write_data(0x01); 
	tft_write_data(0x2C); 
	tft_write_data(0x2D); 

	tft_write_command(0xB3); 
	tft_write_data(0x01); 
	tft_write_data(0x2C); 
	tft_write_data(0x2D); 
	tft_write_data(0x01); 
	tft_write_data(0x2C); 
	tft_write_data(0x2D); 
	
	tft_write_command(0xB4); //Column inversion 
	tft_write_data(0x07); 
	
	//ST7735R Power Sequence
	tft_write_command(0xC0); 
	tft_write_data(0xA2); 
	tft_write_data(0x02); 
	tft_write_data(0x84); 
	tft_write_command(0xC1); 
	tft_write_data(0xC5); 

	tft_write_command(0xC2); 
	tft_write_data(0x0A); 
	tft_write_data(0x00); 

	tft_write_command(0xC3); 
	tft_write_data(0x8A); 
	tft_write_data(0x2A); 
	tft_write_command(0xC4); 
	tft_write_data(0x8A); 
	tft_write_data(0xEE); 
	
	tft_write_command(0xC5); //VCOM 
	tft_write_data(0x0E); 
	
	tft_write_command(0x36); //MX, MY, RGB mode 
	tft_write_data(0xC0); 
	
	//ST7735R Gamma Sequence
	tft_write_command(0xe0); 
	tft_write_data(0x0f); 
	tft_write_data(0x1a); 
	tft_write_data(0x0f); 
	tft_write_data(0x18); 
	tft_write_data(0x2f); 
	tft_write_data(0x28); 
	tft_write_data(0x20); 
	tft_write_data(0x22); 
	tft_write_data(0x1f); 
	tft_write_data(0x1b); 
	tft_write_data(0x23); 
	tft_write_data(0x37); 
	tft_write_data(0x00); 	
	tft_write_data(0x07); 
	tft_write_data(0x02); 
	tft_write_data(0x10); 

	tft_write_command(0xe1); 
	tft_write_data(0x0f); 
	tft_write_data(0x1b); 
	tft_write_data(0x0f); 
	tft_write_data(0x17); 
	tft_write_data(0x33); 
	tft_write_data(0x2c); 
	tft_write_data(0x29); 
	tft_write_data(0x2e); 
	tft_write_data(0x30); 
	tft_write_data(0x30); 
	tft_write_data(0x39); 
	tft_write_data(0x3f); 
	tft_write_data(0x00); 
	tft_write_data(0x07); 
	tft_write_data(0x03); 
	tft_write_data(0x10);  
	
	tft_write_command(0x2a);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x7f);

	tft_write_command(0x2b);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x9f);
	
	tft_write_command(0xF0); //Enable test command  
	tft_write_data(0x01); 
	tft_write_command(0xF6); //Disable ram power save mode 
	tft_write_data(0x00); 
	
	tft_write_command(0x3A); //65k mode 
	tft_write_data(0x05); 
	
	tft_write_command(0x29);//Display on
	
	curr_dir = dir;
	curr_bg_color = bg_color;
	curr_text_color = text_color;
	tft_clear();
}

void tft_set_bg_color(u16 color) {
	curr_bg_color = color;
	tft_clear();
}
	
void tft_set_text_color(u16 color) {
	curr_text_color = color;
}

void tft_set_font(const FONT_INFO *font) {
	curr_font = font;
}

void tft_setregion(u16 x1, u16 y1, u16 x2, u16 y2) {		
	tft_write_command(0x2a);
	tft_write_data(0x00);
	tft_write_data(x1+2);
	tft_write_data(0x00);
	tft_write_data(x2+2);

	tft_write_command(0x2b);
	tft_write_data(0x00);
	tft_write_data(y1+1);
	tft_write_data(0x00);
	tft_write_data(y2+1);
	
	tft_write_command(0x2c);
}

void tft_put_pixel(u8 x, u8 y, u16 color) {
	tft_change_dir(&x, &y, curr_dir);
	tft_setregion(x, y, x+1, y+1);
	tft_write_data(color >> 8);
	tft_write_data(color);
}

void tft_clear() {	
	u8 i,m;
	tft_setregion(0, 0, X_MAX_PIXEL-1, Y_MAX_PIXEL-1);
	tft_write_command(0x2C);
	for(i=0;i<X_MAX_PIXEL;i++)
		for(m=0;m<Y_MAX_PIXEL;m++) {
			tft_write_data(curr_bg_color >> 8);
			tft_write_data(curr_bg_color);
		}
}

void tft_prints(u8 x, u8 y, const char * pstr, ...) {
	u8 buf[256];
	u8* fp = NULL;
	
	va_list arglist;
	va_start(arglist, pstr);
	vsprintf((char*)buf, (const char*)pstr, arglist);
	va_end(arglist);
	
	fp = buf;
	while (*fp) {
		char start = curr_font->startChar;
		FONT_CHAR_INFO char_info = curr_font->charInfo[*fp - start];
		u16 offset = char_info.offset;
		u16 width = char_info.widthBits;
		u8 height = curr_font->heightPages;
		
		for (u8 chy=0; chy<height; chy++) {
			for (u8 blk=0; blk<width; blk+=8) {
				for (u8 chx=0; chx<(width-blk); chx++) {
					u8 pixel = curr_font->data[offset+chy*(width/8+1)+blk/8] & (0x80 >> chx);
					if (pixel)
						tft_put_pixel(x+chx+blk, y+chy, curr_text_color);
				}
			}
		}
		x += (width+1);
		fp++;
	}
}

void tft_fill_area(u8 x, u8 y, u8 w, u8 h, u16 color) {
	for (u8 chx=0; chx<w; chx++) {
		for (u8 chy=0; chy<h; chy++) {
			tft_put_pixel(x+chx, y+chy, color);
		}
	}
}

void tft_print_image(u8 x, u8 y, tImage image) {
	for (u8 chx=0; chx<image.width; chx++) {
		for (u8 chy=0; chy<image.height; chy++) {
			tft_put_pixel(x+chx, y+chy, image.data[chy*image.width + chx]);
		}
	}
}
