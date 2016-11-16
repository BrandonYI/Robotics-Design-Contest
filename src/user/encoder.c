#include "encoder.h"

void encoder_init(Encoder* encoder, ENCODER_ID id)
{
	if (id == ENCODER_LEFT)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	    GPIO_InitTypeDef GPIO_InitStructure;
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	    GPIO_Init(GPIOB, &GPIO_InitStructure);

	    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	    TIM_TimeBaseStructure.TIM_Prescaler = 0;
	    TIM_TimeBaseStructure.TIM_Period = 65535; // Maximal
	    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	    // TIM_EncoderMode_TI1: Counter counts on TI1FP1 edge depending on TI2FP2 level.
	    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	    TIM_Cmd(TIM4, ENABLE);
	}
	if (id == ENCODER_RIGHT)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	    GPIO_InitTypeDef GPIO_InitStructure;
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	    GPIO_Init(GPIOC, &GPIO_InitStructure);


	    GPIO_PinRemapConfig( GPIO_FullRemap_TIM3, ENABLE );

	    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	    TIM_TimeBaseStructure.TIM_Prescaler = 0;
	    TIM_TimeBaseStructure.TIM_Period = 65535; // Maximal
	    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	    // TIM_EncoderMode_TI1: Counter counts on TI1FP1 edge depending on TI2FP2 level.
	    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	    TIM_Cmd(TIM3, ENABLE);
	}
	encoder->id = id;
	encoder->prev = 0;
	encoder->current = 0;
}

void encoder_update(Encoder* encoder)
{
	encoder->prev = encoder->current;
	encoder->current = 0;
	if (encoder->id == ENCODER_LEFT)
	{
		encoder->current = TIM3->CNT;
	}
	if (encoder->id == ENCODER_RIGHT)
	{
		encoder->current = TIM4->CNT;
	}
	//handle overflow
	int overflow = 0;
	if (encoder->prev - encoder->current > 25000) { //jumped gap from 65534 ... 65535 ... 0 ... 1 ... 2
        overflow = 65535;
    } else if (encoder->prev - encoder->current < -25000) { //jumped gap from 2 ... 1 ... 0 ... 65535 ... 65534
        overflow = -65535;
    }
    encoder->current += overflow + (encoder->current - encoder->prev);
}

int get_encoder_value(Encoder* encoder)
{
	return encoder->current;
}
