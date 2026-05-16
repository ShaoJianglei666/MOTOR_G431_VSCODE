/**********************************************************
led.c：运行LED指示灯的控制
**********************************************************/

#include "stm32g4xx_hal.h"
#include "led.h"

static uint16_t led_freq;
static uint16_t led_cnt;

/**********************************************************
void LedInit(void)
Function: LED初始化（闪烁频率、计数器）
caller: 在main初始化时调用
**********************************************************/
void LedInit(void)
{
	/* 初始闪烁频率为每秒2次 */
	led_freq = LED_FREQ_2HZ;
	led_cnt = 0;
}

/**********************************************************
void LedFreqSet(uint16_t freq)
Function: LED闪烁频率设置
**********************************************************/
void LedFreqSet(uint16_t freq)
{
	led_freq = freq;
}

/**********************************************************
void LedFlashing(void)
Function: LED闪烁运行；根据闪烁频率进行计数，周期到则翻转引脚电平
caller: 在中频任务SysTick_Handler中调用，执行频率500Hz
**********************************************************/
void LedFlashing(void)
{
	led_cnt++;
	if(led_cnt >= led_freq)
	{
		led_cnt = 0;
		HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
	}
}

