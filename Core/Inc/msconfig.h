#ifndef MSCONFIG_H_
#define	MSCONFIG_H_

#define	DIR_POSITIVE			1		/* 正转 */
#define	DIR_NEGATIVE			-1		/* 反转 */

#define	SPD_SAMPLE_FREQ			500		/* 速度采样频率：Hz */
#ifndef PWM_PERIOD_CYCLES
//#define	PWM_PERIOD_CYCLES		17000	/*7200*/	/* PWM周期：clk */
#endif
#define	PWM_PERIOD_CYCLES_HALF	(PWM_PERIOD_CYCLES/2)
#define PWM_FREQUENCY						(170000000/PWM_PERIOD_CYCLES)

#define MAX_SPEED_RPM       	2000	/* 最大机械速度：rpm */
#define MIN_SPEED_RPM       	200		/* 最小机械速度：rpm */

#define	MIN_STO_SPD				750		/* 无感最小机械速度：rpm */
#define MC_PI					3.14159265358979323846f

#define ADC_TO_AMP 				(MAX_CURRENT / 2047.0f)	/* ADC采集与实际安培之间的转换系数 */

#define ABS(X)              ( ( (X) >= 0 ) ? (X) : -(X) )
#define sat(x,ll,ul) ( (x) > (ul) ) ? (ul) : ( ( (x) < (ll) ) ? (ll) : (x) )

#endif

