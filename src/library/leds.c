#include "leds.h"

GPIO_TypeDef* led_ports[3] = {GPIOA, GPIOB, GPIOB};
uc16 led_pins[3] = {GPIO_Pin_15, GPIO_Pin_3, GPIO_Pin_4};

/**
  * @brief  Initialization of LED GPIO pins
  * @param  None
  * @retval None
  */
void led_init() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);
	GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	GPIO_ResetBits(GPIOB, GPIO_Pin_4);
}

/**
  * @brief  Turn off a LED
  * @param  id: LED to turn off
  * @retval None
  */
void led_off(LED_ID id) {
	GPIO_ResetBits(led_ports[id], led_pins[id]);
}

/**
  * @brief  Turn on a LED
  * @param  id: LED to turn on
  * @retval None
  */
void led_on(LED_ID id) {
	GPIO_SetBits(led_ports[id], led_pins[id]);
}

/**
  * @brief  Toggle the state of a LED
  * @param  id: LED to toggle
  * @retval None
  */
void led_toggle(LED_ID id) {
	static u8 state[3] = {0};
	
	if (state[id])
		GPIO_ResetBits(led_ports[id], led_pins[id]);
	else
		GPIO_SetBits(led_ports[id], led_pins[id]);
	state[id] = !state[id];
}
