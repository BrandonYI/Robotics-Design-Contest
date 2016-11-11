#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"	  
#include "stm32f10x_tim.h"

void init_encoder_left(void);
void init_encoder_right(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void TIM3_Configuration(void);
void TIM4_Configuration(void);

#endif
