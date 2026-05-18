/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — IF 开环精简版
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
#include "FOC.h"
#include "key.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPEED_RUN  1200    /* 目标转速 (RPM) */
#define SPEED_STEP_RPM  50.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t s_u8MotorRunning = 0;   /* 电机运行标志: 0=停止, 1=运行 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Motor_Start(void);
static void Motor_Stop(void);
static uint16_t ADC_InjectedReadOnce(ADC_HandleTypeDef *hadc);
static void VOFA_SendTelemetry(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  启动电机：开启 PWM + ADC 中断 + 设定目标转速
  */
static void Motor_Start(void)
{
    if (s_u8MotorRunning) return;
    s_u8MotorRunning = 1;

    FOC_Init();

    /* 上桥前先给三相 50% 占空比，避免 PWM 刚启动时沿用 CCR=0。 */
    TIM1->CCR1 = PWM_HALF_CYCLE;
    TIM1->CCR2 = PWM_HALF_CYCLE;
    TIM1->CCR3 = PWM_HALF_CYCLE;
    TIM1->CNT  = 0;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

    /* 启动 TIM1 六路 PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* 启动 ADC 注入组（TIM1_CH4 硬件触发） */
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);

    /* 使能 TIM1 更新中断（10kHz FOC 控制环） */
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

    /* 设定目标转速 → 状态机自动进入 IF 开环 */
    g_stCtrl.f32TargetRpm = (float)SPEED_RUN;
}

/**
  * @brief  停止电机：关 PWM + 关中断 + 复位 FOC 状态
  */
static void Motor_Stop(void)
{
    if (!s_u8MotorRunning) return;
    s_u8MotorRunning = 0;

    /* 先通知状态机停止 */
    g_stCtrl.f32TargetRpm = 0.0f;

    /* 关闭 TIM1 更新中断 */
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);

    TIM1->CCR1 = PWM_HALF_CYCLE;
    TIM1->CCR2 = PWM_HALF_CYCLE;
    TIM1->CCR3 = PWM_HALF_CYCLE;

    /* 关闭 ADC 注入组中断 */
    HAL_ADCEx_InjectedStop_IT(&hadc1);
    HAL_ADCEx_InjectedStop_IT(&hadc2);

    /* 关闭 TIM1 六路 PWM 输出 */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    /* FOC 状态机复位 */
    FOC_Init();
}

static uint16_t ADC_InjectedReadOnce(ADC_HandleTypeDef *hadc)
{
    uint32_t saved_jsqr = hadc->Instance->JSQR;
    uint16_t value;

    hadc->Instance->JSQR &= ~ADC_JSQR_JEXTEN;
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC | ADC_FLAG_JEOS);

    if (HAL_ADCEx_InjectedStart(hadc) == HAL_OK)
    {
        if (HAL_ADCEx_InjectedPollForConversion(hadc, 10U) != HAL_OK)
        {
            Error_Handler();
        }
        value = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        HAL_ADCEx_InjectedStop(hadc);
    }
    else
    {
        Error_Handler();
        value = 0U;
    }

    hadc->Instance->JSQR = saved_jsqr;
    return value;
}

static int32_t VOFA_FloatToMilli(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value * 1000.0f + 0.5f);
    }

    return (int32_t)(value * 1000.0f - 0.5f);
}

static int VOFA_AppendMilli(char *buf, int pos, int size, float value)
{
    int32_t scaled = VOFA_FloatToMilli(value);
    int32_t abs_scaled = scaled;
    const char *sign = "";

    if (scaled < 0)
    {
        sign = "-";
        abs_scaled = -scaled;
    }

    return pos + snprintf(&buf[pos], (size_t)(size - pos), "%s%ld.%03ld",
                          sign,
                          (long)(abs_scaled / 1000),
                          (long)(abs_scaled % 1000));
}

static float VOFA_RadToDeg(float rad)
{
    return rad * 57.2957795f;
}

static float VOFA_WrapDeg180(float deg)
{
    while (deg >= 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static void VOFA_SendTelemetry(void)
{
    char buf[384];
    int pos = 0;
    float theta_open_deg = VOFA_RadToDeg(g_stMotor.f32Theta);
    float theta_obs_deg = VOFA_RadToDeg(g_stLuenberger.f32ThetaObs);
    float theta_err_deg = VOFA_WrapDeg180(theta_obs_deg - theta_open_deg);

    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "foc:");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stCtrl.f32TargetRpm);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stCtrl.f32RpmRamp);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stCtrl.f32IdRef);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stCtrl.f32IqRef);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stMotor.f32Id);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stMotor.f32Iq);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stMotor.f32UdRef);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stMotor.f32UqRef);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), theta_open_deg);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), theta_obs_deg);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), theta_err_deg);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stLuenberger.f32SpeedObs);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stCtrl.f32RampedTargetRpm);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stPiSpeed.fOutPrev);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stCtrl.f32SpdProportional);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stCtrl.f32SpdIntegral);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stCtrl.f32IqBase);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",%u", (unsigned int)g_stCtrl.u16DiagStage);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "\n");

    if ((pos > 0) && (pos < (int)sizeof(buf)))
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)pos, 10U);
    }
}

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
  HAL_Init();
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);
  HAL_Delay(10);

  /*--- FOC 初始化 ---*/
  FOC_Init();

  /* 电流零点校准（PWM 未启动时软件触发注入转换取平均值） */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_Delay(100);
  {
      int32_t sum_a = 0, sum_b = 0;
      for (int i = 0; i < 128; i++) {
          sum_a += ADC_InjectedReadOnce(&hadc1);
          sum_b += ADC_InjectedReadOnce(&hadc2);
      }
      FOC_fIaOffsetAdc = (float)sum_a / 128.0f;
      FOC_fIbOffsetAdc = (float)sum_b / 128.0f;
  }

  /* TIM1 中断优先级（仅配置，不使能，由 Motor_Start 使能） */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*--- 按键扫描 ---*/
    KEY_Scan();

    /*--- 单键启停：释放一次切换一次状态 ---*/
    if (KEY_GetEvent(KEY_ID_RUN) == KEY_EVENT_RELEASE)
    {
        if (s_u8MotorRunning)
        {
            Motor_Stop();
        }
        else
        {
            Motor_Start();
        }
    }

    /*--- 闭环后单击 PC9：目标速度 +50rpm ---*/
    if (KEY_GetEvent(KEY_ID_SPEED_UP) == KEY_EVENT_PRESS)
    {
#if !FOC_FORCE_OPEN_LOOP
        if (s_u8MotorRunning && (g_stCtrl.eMode == FOC_MODE_CLOSED_LOOP))
        {
            float fNewTarget = g_stCtrl.f32TargetRpm + SPEED_STEP_RPM;
            if (fNewTarget > FOC_OBS_MAX_SPEED_RPM)
            {
                fNewTarget = FOC_OBS_MAX_SPEED_RPM;
            }
            g_stCtrl.f32TargetRpm = fNewTarget;
        }
#endif
    }

    /*--- LED1 (PB12) 以 1Hz 闪烁 ---*/
    {
        static uint32_t s_u32LedTick = 0;
        if (HAL_GetTick() - s_u32LedTick >= 500U)
        {
            s_u32LedTick = HAL_GetTick();
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        }
    }

    /*--- VOFA FireWater telemetry, 500Hz ---*/
    {
        static uint32_t s_u32VofaTick = 0;
        if (HAL_GetTick() - s_u32VofaTick >= 2U)
        {
            s_u32VofaTick = HAL_GetTick();
            VOFA_SendTelemetry();
        }
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
