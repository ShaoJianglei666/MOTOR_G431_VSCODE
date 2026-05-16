/**
  ******************************************************************************
  * @file    FOC.h
  * @brief   FOC 开环 IF 控制 — 精简版头文件 【v3.0】
  *          纯 IF 开环：角度开环积分 + 电流闭环 PI + SVPWM 输出
  ******************************************************************************
  * @attention
  *   - 角度 θ: 浮点弧度 [0, 2π)，由转速积分生成（不依赖观测器）
  *   - 电压 V: 标幺值 [-1.0, 1.0] pu（1.0 pu = Vdc/√3）
  *   - 电流 I: 标幺值 [-1.0, 1.0] pu（1.0 pu = FOC_BASE_CURRENT_A）
  *   - 转速:   float RPM
  ******************************************************************************
  */

#ifndef __FOC_H
#define __FOC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include "Motor_Param.h"

/* Exported constants --------------------------------------------------------*/

/** @brief 圆周率 π */
#define FOC_MATH_PI                 3.14159265358979323846f
/** @brief 2π */
#define FOC_2PI                     (2.0f * FOC_MATH_PI)
/** @brief π/2 */
#define FOC_PI_2                    (FOC_MATH_PI * 0.5f)
/** @brief √3 */
#define FOC_SQRT3                   1.7320508075688772f
/** @brief 1/√3 */
#define FOC_INV_SQRT3               0.5773502691896257f

/*--- PWM 配置（TIM1 中心对齐模式，ARR=8500） ---*/
#define FOC_PWM_PERIOD_CYCLES       17000U
#define FOC_PWM_ARR                 (FOC_PWM_PERIOD_CYCLES / 2U)
#define PWM_HALF_CYCLE              (FOC_PWM_PERIOD_CYCLES / 4U)

/*--- FOC 控制频率 ---*/
#define FOC_CTRL_FREQ               10000U
#define FOC_CTRL_TS                 (1.0f / (float)FOC_CTRL_FREQ)

/*--- 电压限幅 ---*/
#define FOC_VOLTAGE_MAX_PU          0.60f
#define FOC_VOLTAGE_MIN_PU          0.06f
#define FOC_VOLTAGE_MARGIN_PU       0.04f
#define FOC_PI_CORRECTION_MAX_PU    0.06f
#define FOC_VOLTAGE_1PU_V           (MOTOR_BUS_VOLTAGE / FOC_SQRT3)

/*--- 电流采样 ---*/
#define FOC_ADC_CURRENT_SCALE       2048.0f
#define FOC_CURRENT_POLARITY_A      1.0f
#define FOC_CURRENT_POLARITY_B      1.0f

/*--- IF 开环参数 ---*/
#define FOC_IF_ALIGN_ID             0.015f     /**< 对齐 Id (pu), 从小开始防啸叫 */
#define FOC_IF_STARTUP_IQ           0.01f     /**< 运行 Iq (pu) */
#define FOC_IF_ALIGN_FRAMES         3000U     /**< 对齐阶段帧数 (300ms@10kHz) */
#define FOC_IF_TRANSITION_FRAMES    500U      /**< Id->Iq 过渡帧数 (50ms@10kHz) */
#define FOC_IF_RAMP_STEP_RPM        20.0f     /**< 加速步长 (RPM), 放慢防失步 */
#define FOC_IF_RAMP_INTERVAL        400U      /**< 加速间隔 (帧, 40ms@10kHz) */

/*--- 电压前馈参数（基于 Motor_Param.h） ---*/
/** @brief 电流基值 (A) */
#define FOC_BASE_CURRENT_A          2.0f
/** @brief 电压基值 (V) = Vdc/√3 */
#define FOC_BASE_VOLTAGE_V          (MOTOR_BUS_VOLTAGE / FOC_SQRT3)
/** @brief 电气角速度基值 (rad/s) */
#define FOC_BASE_OMEGA_RADS         628.3185f  /* 2π×100 */
/** @brief 定子电阻 pu */
#define FOC_RS_PU                   (MOTOR_PHASE_RESISTANCE * FOC_BASE_CURRENT_A / FOC_BASE_VOLTAGE_V)
/** @brief 永磁磁链 pu: ψf = Ke_phase_peak / (1000×Vb/ωb) */
#define FOC_PSIF_PU                 ((MOTOR_BEMF_CONST_V_LL / 2.0f) / 1000.0f \
                                     * FOC_BASE_OMEGA_RADS / FOC_BASE_VOLTAGE_V)
/** @brief 机械转速对应的相反电势峰值 pu */
#define FOC_BEMF_PU_PER_RPM         ((MOTOR_BEMF_CONST_V_LL / FOC_SQRT3) \
                                     / (1000.0f * FOC_BASE_VOLTAGE_V))

/*--- PI 电流环增益（小电流电机降低增益防振荡） ---*/
#define FOC_PI_ID_KP                0.05f
#define FOC_PI_ID_KI                0.001f
#define FOC_PI_IQ_KP                0.05f
#define FOC_PI_IQ_KI                0.001f

/*--- 正弦查找表 ---*/
#define FOC_SIN_TABLE_SIZE          256U
#define FOC_ANGLE_TO_IDX_SCALE      40.743665f

/* Exported types ------------------------------------------------------------*/

/**
  * @brief 电机电气状态（精简版）
  */
typedef struct
{
    float    f32Theta;          /**< 电角度 (rad) */
    float    f32UdRef;          /**< d轴电压指令 (pu) */
    float    f32UqRef;          /**< q轴电压指令 (pu) */
    float    f32Valpha;         /**< α轴电压 (pu) */
    float    f32Vbeta;          /**< β轴电压 (pu) */
    uint16_t u16Ta;             /**< A相PWM比较值 */
    uint16_t u16Tb;             /**< B相PWM比较值 */
    uint16_t u16Tc;             /**< C相PWM比较值 */
    float    f32Ia;             /**< A相电流 (pu) */
    float    f32Ib;             /**< B相电流 (pu) */
    float    f32Ialpha;         /**< α轴电流 (pu) */
    float    f32Ibeta;          /**< β轴电流 (pu) */
    float    f32Id;             /**< d轴电流 (pu) */
    float    f32Iq;             /**< q轴电流 (pu) */
} FOC_MotorState;

/**
  * @brief 控制模式（精简版）
  */
typedef enum
{
    FOC_MODE_STOP        = 0,   /**< 停止 */
    FOC_MODE_IF_OPENLOOP = 1    /**< IF 开环运行 */
} FOC_Mode;

/**
  * @brief 控制状态（精简版）
  */
typedef struct
{
    FOC_Mode  eMode;            /**< 当前模式 */
    float     f32TargetRpm;     /**< 目标转速 (RPM) */
    float     f32IdRef;         /**< d轴电流指令 (pu) */
    float     f32IqRef;         /**< q轴电流指令 (pu) */
    float     f32RpmRamp;       /**< 斜坡当前转速 (RPM) */
    uint32_t  u32RampCount;     /**< 斜坡计数器（帧） */
    uint32_t  u32RunFrames;     /**< 总运行帧数（诊断） */
} FOC_ControlState;

/**
  * @brief PI 控制器
  */
typedef struct
{
    float fKp;
    float fKi;
    float fIntegral;
    float fOutMax;
    float fOutMin;
} FOC_PI;

/* Exported variables --------------------------------------------------------*/

extern FOC_MotorState    g_stMotor;
extern FOC_ControlState  g_stCtrl;
extern FOC_PI            g_stPiId;
extern FOC_PI            g_stPiIq;
extern float             FOC_fIaOffsetAdc;
extern float             FOC_fIbOffsetAdc;
extern const float       FOC_SinTable_F32[FOC_SIN_TABLE_SIZE];

/* Exported function prototypes ----------------------------------------------*/

void  FOC_Init(void);
void  FOC_ControlStep(void);
void  FOC_GetPhaseCurrent(void);

void  FOC_Svpwm(float fValpha, float fVbeta,
                uint16_t *pu16Ta, uint16_t *pu16Tb, uint16_t *pu16Tc);
void  FOC_Clarke(float fIa, float fIb, float *pfIalpha, float *pfIbeta);
void  FOC_Park(float fIalpha, float fIbeta, float fTheta, float *pfId, float *pfIq);
void  FOC_InvPark(float fVd, float fVq, float fTheta, float *pfValpha, float *pfVbeta);
float FOC_PI_Run(FOC_PI *pstPi, float fRef, float fFb);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H */
