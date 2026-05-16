#ifndef HALL_H
#define HALL_H

#define	MAX_RELIABLE_MEC_SPEED_RPM	(uint16_t)(1.15*MAX_SPEED_RPM)
#define	MIN_RELIABLE_MEC_SPEED_RPM	(uint16_t)(MIN_SPEED_RPM)
#define	MEASUREMENT_FREQUENCY		PWM_FREQUENCY
#define DPP_CONV_FACTOR 			65536

/* hall缓冲区最大尺寸 */
#define HALL_SPEED_FIFO_SIZE  		((uint8_t)18)

/* 速度计时单位 */
#define SPEED_UNIT 					60	// rpm

/* hall传感器的放置方式 */
#define DEGREES_120					0u	// 120°放置
#define DEGREES_60					1u	// 60°放置

/* 本代码当前适配的电机hall均为120°放置 */
#define HALL_SENSORS_PLACEMENT  	DEGREES_120

/* hall相位偏移 */
/* 电机TG5P60的hall相位偏移 */
#define	HALL_BEMF_TG5P60			0	// 0°	//320°
/* 电机JSF630、JSF840的hall相位偏移 */
#define	HALL_BEMF_JSF630			35	// 35°
/* 电机TB2P的hall相位偏移 */
#define	HALL_BEMF_TB2P				35	// 35°

/* 计算平均速度时用到的缓冲区尺寸 */
#define HALL_AVERAGING_FIFO_DEPTH	6	// 缓冲区最大18，这里用了6个

/* 是否启用MTPA功能 */
#define HALL_MTPA  					0
/* 是否启用动态调整预分频功能 */
#define	HALL_DYNA_PRSC				1
/* 是否启动堵转前馈补偿功能 */
#define	HALL_STALL_FFC				1

/* hall定时器时钟频率 */
#define HALL_TIM_CLK       			170000000uL//72000000uL

/* 3个hall传感器接入mcu的端口与引脚定义 */
#define	H1_GPIO_PORT		((GPIO_TypeDef *)GPIOB)
#define	H2_GPIO_PORT		((GPIO_TypeDef *)GPIOB)
#define	H3_GPIO_PORT		((GPIO_TypeDef *)GPIOC)
#define	H1_PIN				GPIO_PIN_4
#define	H2_PIN				GPIO_PIN_5
#define	H3_PIN				GPIO_PIN_8

/* hall自学习状态 */
enum
{
	HALL_LEARN_IDLE = 0,	// 空闲
	HALL_LEARN_PREPARE, 	// 准备
	HALL_LEARN_RUN,			// 学习中
	HALL_LEARN_COMP,		// 完成
	HALL_LEARN_MAX,
};

/* hall堵转标志 */
enum
{
	HALL_STALL_NO = 0,	// 无堵转
	HALL_STALL_OVF, 	// 检测到超时
	HALL_STALL_REVERSE,	// 检测到反向
	HALL_STALL_MAX,
};

/* hall自学习准备时间：为了避开IF加速时间段 */
/* IF_ACC_DURATION + 1000：IF加速持续时间上增加1000ms */
/* /1000：1s = 1000ms */
#define HALL_LEARN_PREPARE_TIME	((IF_ACC_DURATION + 1000) * SPD_SAMPLE_FREQ / 1000)

typedef struct
{
  /* SW Settings */
  uint8_t  sensor_placement;	/* hall放置方式：120°或60° */
  int16_t  hall_bemf;			/* 相位偏移 */
  uint16_t spd_samp_freq;		/* 速度采样频率：500Hz */
  uint8_t  buff_size;			/* 速度缓冲区尺寸 */

  /* HW Settings */
  uint32_t tim_clk_freq;		/* hall使用的TIM时钟频率：72000000 */
  TIM_TypeDef * TIMx;			/* hall使用的TIM：TIM4 */

  /* hall运行的相关标志 */
  uint8_t operational;			/* hall是否可用 */
  uint8_t run_flag;				/* 电机运行标志，对应motor_start */
  uint8_t stall;				/* hall是否堵转 */

  /* 计数器相关 */
  uint8_t ratio_dec;			/* TIM预分频减小标志 */
  uint8_t ratio_inc;			/* TIM预分频增加标志 */
  uint16_t ovf_num;				/* 计数器溢出次数 */

  /* 速度缓冲相关 */
  uint8_t first_capt;			/* 第一次捕获hall状态标志 */
  uint8_t buf_filled;			/* 速度缓冲区已填满 */
  uint8_t buf_idx;				/* 缓冲区索引 */
  int32_t state_period[HALL_SPEED_FIFO_SIZE];	/* hall状态时长缓冲区 */                                 
  int32_t period_sum;			/* 缓冲区中hall状态时长的累加和 */
  uint16_t capt_cnt;			/* 从HallStart开始捕获的hall状态计数器 */
  
  /* hall状态及对应角度 */
  uint8_t cur_state;			/* hall状态：共6个状态 */
  uint8_t state_tab[6];			/* 按转动时触发顺序记录hall的6个状态，每个状态在数组中的位置不是固定的 */
  int16_t state_theta_tab[6];	/* 记录state_tab中每个hall状态对应的电角度，单位s16 */


  /* 速度、角度、方向相关 */
  int8_t direction;				/* 转动方向：正转、反转 */
  int8_t direction_ref;			/* 参考转动方向：正转、反转 */
  int16_t avr_spd_dpp;			/* 平均电气速度：s16Degree/PWM周期 */
  int16_t spd_dpp_e;			/* 电气速度：s16Degree/PWM周期 */
  int16_t prev_rotor_freq;
  int16_t comp_spd;				/* 补偿速度 */
  int16_t delta_theat;			/* 测量电角度与输出电角度差值 */
  int16_t measured_theat_e;		/* 测量电角度：hall状态被捕获时的测量的电角度 */
  int16_t theat_e;				/* 输出电角度：s16Degree */

  /* 预定义参数 */
  uint16_t max_ratio;			/* HallTimeout对应的计数器溢出/更新次数，也是预分频的最大值 */
  uint16_t sat_speed;			/* 速度饱和上限 */
  uint32_t PseudoFreqConv;		/* 计算平均速度的系数 */
  uint32_t max_period;			/* hall状态最大时长：clk；该值和HallTimeout相契合 */
  uint32_t min_period;			/* hall状态最小时长：clk；该值和最低速度相契合 */
  uint16_t hall_timeout;		/* hall超时值：ms；在该时间内应该捕获到hall状态，但没有捕获到 */
  uint16_t ovf_freq;			/* 计数器溢出频率：72000000/65536；即每秒溢出OvfFreq次 */
  uint16_t pwmnum_per_spdsamp;	/* 每个速度采样周期PWM运行次数 */

  /* 控制策略 */
  uint8_t hall_mtpa;

  /* hall自学习相关 */
  uint8_t learn_start;			/* 启动hall自学习 */
  uint8_t learn_sts;			/* hall自学习的状态：空闲、准备、运行、完成 */
  uint8_t learn_idx;			/* hall自学习中记录hall状态的索引 */
  uint32_t learn_start_time;	/* hall自学习的启动时间，并不是learn_start从0到1的时间 */
  uint8_t learn_cnt[6];			/* hall自学习中每个状态被学习的次数 */
  int32_t learn_theta_sum[6];	/* 每个状态多次学习的角度之和，用于求平均值获得更加精确的角度 */
} HallHandle_t;

HallHandle_t *HallGetHandle(void);
void HallInit(void);
uint8_t HallReadState(void);
uint32_t HallReadPeriod(void);
void HallInitElAngle(void);
void HallChangeMeasure(uint8_t pre_state, uint8_t cur_state);
void HallStart(int8_t dir);
void HallStop(void);
void HallTimCCIRQHandler(void);
void HallTimUPIRQHandler(void);
uint8_t HallCalcAngle(float *theta_e);
uint8_t HallCalcSpdPos(int16_t *mec_spd, int32_t *mec_pos);
uint8_t HallGetLearnStart(void);
void HallSetLearnStart(uint8_t on);
void HallLearnExec(void);
void HallStallReset(void);

#endif

