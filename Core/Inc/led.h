#ifndef LED_H_
#define	LED_H_

/* 运行LED端口、PIN定义 */
#define	LED_PIN		GPIO_PIN_12
#define	LED_GPIO_PORT	((GPIO_TypeDef *)GPIOB)

/* LED闪烁频率定义 */
#define	LED_FREQ_1HZ	250		// 每秒闪烁1次，250*2ms
#define	LED_FREQ_2HZ	125		// 每秒闪烁2次，125*2ms
#define	LED_FREQ_5HZ	50		// 每秒闪烁5次，50*2ms
#define	LED_FREQ_10HZ	25		// 每秒闪烁10次，25*2ms

void LedInit(void);
void LedFreqSet(uint16_t freq);
void LedFlashing(void);

#endif

