/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/** @brief IF 开环调试模式：注释此行恢复完整的 Imperix 五阶段闭环 */
#define FOC_IF_OPENLOOP_DEBUG

extern uint8_t Uart3_Rx_F;
extern uint8_t Uart3_Rx_Data_Len;
extern uint8_t Uart3_Rx_Buf[];

#include "FOC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define FOC_DEBUG_TEST 0



/* 按键消抖时间 (ms) — 无硬件电容，适当加大 */
#define KEY_DEBOUNCE_MS     50

/* 速度调节参数 */
#define SPEED_RUN           1500    /* 按 RUN 的目标转速 */
#define SPEED_MIN           300     /* DOWN 下限 */
#define SPEED_MAX           3000    /* UP 上限 */
#define SPEED_STEP          100     /* UP/DOWN 步长 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#ifdef FOC_DEBUG_TEST
/* 按键消抖状态：1=检测到一次有效按下（待处理），由主循环清0 */
static uint8_t s_u8KeyRunPressed   = 0;
static uint8_t s_u8KeyStopPressed  = 0;
static uint8_t s_u8KeyUpPressed    = 0;
static uint8_t s_u8KeyDownPressed  = 0;
#endif
/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART3_ProcessReceivedData(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef FOC_DEBUG_TEST

/**
  * @brief  FOC 状态发送任务（主循环中调用，浮点版）
  */
static void FOC_State_Send(void)
{
    static const uint32_t JUSTFLOAT_TAIL = 0x7F800000UL;
    float fSendBuf[14];

    fSendBuf[0]  = g_stMotor.f32Theta;                /* 电角度 θ (rad) */
    fSendBuf[1]  = g_stMotor.f32Id;                   /* Id (pu) */
    fSendBuf[2]  = g_stMotor.f32Iq;                   /* Iq (pu) */
    fSendBuf[3]  = g_stCtrl.f32IdRef;                 /* Id_ref (pu) */
    fSendBuf[4]  = g_stCtrl.f32IqRef;                 /* Iq_ref (pu) */
#ifdef FOC_IF_OPENLOOP_DEBUG
    fSendBuf[5]  = g_stMotor.f32Theta;                /* IF开环: θ 参考 (rad) */
    fSendBuf[6]  = g_stCtrl.f32RpmRamp;               /* IF开环: 斜坡转速 (RPM) */
#else
    fSendBuf[5]  = g_stLuenberger.f32ThetaObs;        /* θ̂ (rad) */
    fSendBuf[6]  = g_stLuenberger.f32SpeedObs;        /* 转速 (RPM) */
#endif
    fSendBuf[7]  = (float)(int32_t)g_stCtrl.eMode;    /* 模式 */
    fSendBuf[8]  = g_stCtrl.f32TorqueAngle * 57.29578f; /* 转矩角 (°) */
    fSendBuf[9]  = (float)g_stCtrl.u16BlendCount;     /* 过渡计数 */
    fSendBuf[10] = (float)g_stCtrl.u16DiagTransitionFlag; /* 过渡标志 */
    fSendBuf[11] = g_stPiSpeed.fIntegral;             /* 速度PI积分(I) (pu) */
    fSendBuf[12] = g_stCtrl.f32ThetaErrSave;          /* 速度PI比例(P) (pu) */

    *(uint32_t *)&fSendBuf[13] = JUSTFLOAT_TAIL;
    HAL_UART_Transmit(&huart3, (uint8_t *)fSendBuf, 56, 50);
}

#endif /* FOC_DEBUG_TEST */


#ifdef FOC_DEBUG_TEST

/**
  * @brief  按键扫描（软件消抖，防按键抖动和误触发）
  *         检测下降沿（高→低），连续低电平保持 KEY_DEBOUNCE_MS 后确认一次有效按下。
  *         一旦释放，复位消抖状态，下次按下可再次触发。
  * @note   每个按键单次按下仅触发一次，长按不重复触发。
  */
static void KEY_Scan(void)
{
    uint32_t u32Now = HAL_GetTick();
    uint8_t  u8Val;

    /*--- RUN (PA3) ---*/
    {
        static uint32_t s_u32Tick = 0;
        static uint8_t  s_u8Last  = 1;
        u8Val = (uint8_t)HAL_GPIO_ReadPin(KEY_RUN_PORT, KEY_RUN_PIN);
        if (u8Val == 0) /* 按下 */
        {
            if (s_u8Last == 1) { s_u32Tick = u32Now; }                     /* 下降沿 */
            else if ((u32Now - s_u32Tick) >= KEY_DEBOUNCE_MS && !s_u8KeyRunPressed)
                { s_u8KeyRunPressed = 1; }                                 /* 消抖确认 */
        }
        s_u8Last = u8Val;
    }

    /*--- STOP (PA4) ---*/
    {
        static uint32_t s_u32Tick = 0;
        static uint8_t  s_u8Last  = 1;
        u8Val = (uint8_t)HAL_GPIO_ReadPin(KEY_STOP_PORT, KEY_STOP_PIN);
        if (u8Val == 0)
        {
            if (s_u8Last == 1) { s_u32Tick = u32Now; }
            else if ((u32Now - s_u32Tick) >= KEY_DEBOUNCE_MS && !s_u8KeyStopPressed)
                { s_u8KeyStopPressed = 1; }
        }
        s_u8Last = u8Val;
    }

    /*--- UP (PA5) ---*/
    {
        static uint32_t s_u32Tick = 0;
        static uint8_t  s_u8Last  = 1;
        u8Val = (uint8_t)HAL_GPIO_ReadPin(KEY_UP_PORT, KEY_UP_PIN);
        if (u8Val == 0)
        {
            if (s_u8Last == 1) { s_u32Tick = u32Now; }
            else if ((u32Now - s_u32Tick) >= KEY_DEBOUNCE_MS && !s_u8KeyUpPressed)
                { s_u8KeyUpPressed = 1; }
        }
        s_u8Last = u8Val;
    }

    /*--- DOWN (PB1) ---*/
    {
        static uint32_t s_u32Tick = 0;
        static uint8_t  s_u8Last  = 1;
        u8Val = (uint8_t)HAL_GPIO_ReadPin(KEY_DOWN_PORT, KEY_DOWN_PIN);
        if (u8Val == 0)
        {
            if (s_u8Last == 1) { s_u32Tick = u32Now; }
            else if ((u32Now - s_u32Tick) >= KEY_DEBOUNCE_MS && !s_u8KeyDownPressed)
                { s_u8KeyDownPressed = 1; }
        }
        s_u8Last = u8Val;
    }

    /*--- DIR (PB11)：暂不处理 ---*/
}

#endif /* FOC_DEBUG_TEST */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /*--- FOC 初始化 ---*/
  FOC_Init();

  /* 电流零点校准：在 PWM 未启动时读取 ADC 平均值 */
  HAL_Delay(100);
  {
      int32_t sum_a = 0, sum_b = 0;
      for (int i = 0; i < 100; i++) {
          sum_a += ADC1->JDR1;
          sum_b += ADC2->JDR1;
          HAL_Delay(1);
      }
      FOC_fIaOffsetAdc = (float)sum_a / 100.0f;
      FOC_fIbOffsetAdc = (float)sum_b / 100.0f;
  }



  /*--- 启动 TIM1 六路 PWM 输出 ---*/
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  /*--- 启动 ADC 注入组中断模式（由 TIM1_CH4 硬件触发） ---*/
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart_IT(&hadc2);

  /*--- 使能 TIM1 更新中断（10kHz） ---*/
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);


  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef FOC_DEBUG_TEST
    /*--- 按键扫描（带消抖） ---*/
    KEY_Scan();

    /*--- 按键处理 ---*/
    if (s_u8KeyRunPressed)
    {
        s_u8KeyRunPressed = 0;
        g_stCtrl.f32TargetRpm = (float)SPEED_RUN;
    }

    if (s_u8KeyStopPressed)
    {
        s_u8KeyStopPressed = 0;
        g_stCtrl.f32TargetRpm = 0.0f;
    }

    if (s_u8KeyUpPressed)
    {
        s_u8KeyUpPressed = 0;
        if (g_stCtrl.f32TargetRpm > 0.0f)
        {
            float fNew = g_stCtrl.f32TargetRpm + (float)SPEED_STEP;
            if (fNew > (float)SPEED_MAX) fNew = (float)SPEED_MAX;
            g_stCtrl.f32TargetRpm = fNew;
        }
    }

    if (s_u8KeyDownPressed)
    {
        s_u8KeyDownPressed = 0;
        if (g_stCtrl.f32TargetRpm > 0.0f)
        {
            float fNew = g_stCtrl.f32TargetRpm - (float)SPEED_STEP;
            if (fNew < (float)SPEED_MIN) fNew = (float)SPEED_MIN;
            g_stCtrl.f32TargetRpm = fNew;
        }
    }

    /* 发送 VOFA+ 数据（约 100Hz，避免阻塞主循环） */
    {
        static uint32_t s_u32LastSendTick = 0;
        uint32_t u32Now = HAL_GetTick();
        if (u32Now - s_u32LastSendTick >= 10)
        {
            s_u32LastSendTick = u32Now;
            FOC_State_Send();
        }
    }
#endif
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    /*--- LED1 (PB12) 以 1Hz 闪烁（低电平点亮） ---*/
    static uint32_t s_u32LedTick = 0;
    if (HAL_GetTick() - s_u32LedTick >= 500U)
    {
        s_u32LedTick = HAL_GetTick();
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  UART 命令解析处理
  *         支持命令格式："cmd:param=value\r\n"
  *         当前支持命令：
  *           set:uq=<value>       设置 Iq 电流指令 (Q15)
  *           set:id=<value>       设置 Id 电流指令 (Q15)
  *           set:targetspeed=<rpm> 设置目标转速 (RPM)
  *
  * @note  无回显，不阻塞 VOFA+ 数据流。
  */
void UART3_ProcessReceivedData(void)
{
    if (Uart3_Rx_F == 1)
    {
        Uart3_Rx_F = 0;
        Uart3_Rx_Data_Len = 0;
    }
}

#ifdef FOC_DEBUG_TEST
/**
  * @brief  TIM 周期中断回调
  *         TIM1 更新中断仅用于重装备 ADC 触发，控制环已移至 ADC 中断中。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    (void)htim;
}
#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
