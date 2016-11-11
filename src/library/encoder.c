#include "encoder.h"
void init_encoder_left(void) {
    //tim4
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
void init_encoder_right(void) {
    //tim3
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

/**********************UNUSED***************************/
void RCC_Configuration(void) {
    // clock for GPIO and AFIO (for ReMap)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);


    // clock for TIM3, 4
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
}


void GPIO_Configuration(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // PC.06 TIM3_CH1, PC.07 TIM3_CH2 right tim3
    // PB6 TIM4_CH1, PB7 TIM4_CH2 left tim4

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    GPIO_PinRemapConfig( GPIO_FullRemap_TIM3, ENABLE );        // Map TIM3 to GPIOC
}



void TIM3_Configuration(void) {
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

void TIM4_Configuration(void) {
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

/*****************************************************/

