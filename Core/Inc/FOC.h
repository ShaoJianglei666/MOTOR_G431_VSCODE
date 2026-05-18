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
#define FOC_CURRENT_POLARITY_A     (-1.0f)
#define FOC_CURRENT_POLARITY_B     (-1.0f)

/*--- IF 开环参数 ---*/
#define FOC_FORCE_OPEN_LOOP         0         /**< 1=只运行开环调试, 0=允许切换闭环 */
#define FOC_IF_ALIGN_ID             0.0f      /**< 参考启动阶段不做静止 Id 对齐 */
#define FOC_Q12_TO_PU(x)            ((x) / 4096.0f)
#define FOC_PU_TO_Q12(x)            ((x) * 4096.0f)
#define FOC_IF_IQ_START_Q12         500.0f    /**< Iq 软启动起点 (Q12计数, 约0.24A@2A基值) */
#define FOC_IF_STARTUP_IQ_Q12       300.0f    /**< IF 启动 Iq 参考 (Q12计数, 约0.15A@2A基值) */
#define FOC_IF_IQ_START             FOC_Q12_TO_PU(FOC_IF_IQ_START_Q12)
#define FOC_IF_STARTUP_IQ           FOC_Q12_TO_PU(FOC_IF_STARTUP_IQ_Q12)
#define FOC_CLOSED_IQ_BASE_RATIO    (1.0f / 3.0f) /**< 闭环维持电流目标比例 */
#define FOC_CLOSED_IQ_BASE_TARGET   (FOC_IF_STARTUP_IQ * FOC_CLOSED_IQ_BASE_RATIO)
#define FOC_IF_IQ_RAMP_FRAMES       1000U     /**< Iq 软启动时间 (100ms@10kHz) */
#define FOC_IF_SWITCH_RPM           1000.0f    /**< 本工程闭环切换转速 (RPM) */
#define FOC_IF_SETTLE_FRAMES        20000U    /**< 切换转速恒速等待 (2s@10kHz) */
#define FOC_IF_RAMP_STEP_RPM        50.0f     /**< 加速步长 (RPM) */
#define FOC_IF_RAMP_INTERVAL        800U      /**< 加速间隔 (帧, 80ms@10kHz) */
#define FOC_IF_SPEED_TOL_PCT        10U       /**< 观测速度容差百分比 (±10%) */
#define FOC_IF_STABLE_COUNT_THR     500U      /**< 速度稳定判定连续帧数 (50ms@10kHz) */
#define FOC_TRANSITION_FRAMES       5000U     /**< 角度渐进切换帧数 (500ms@10kHz) */
#define FOC_CLOSED_STABLE_FRAMES    20000U    /**< 闭环切换后失锁屏蔽帧数 (2s@10kHz) */
#define FOC_OBS_LOST_THRESHOLD      5000U     /**< 观测器失锁保护帧数 (500ms@10kHz) */

/*--- 诊断阶段码（串口可视化） ---*/
#define FOC_STAGE_STOP              0U
#define FOC_STAGE_ALIGN             1U        /**< Iq 软启动/起动阶段 */
#define FOC_STAGE_OPEN_LOOP         2U        /**< IF 开环加速/恒速阶段 */
#define FOC_STAGE_TRANSITION        3U        /**< 角度渐变切换阶段 */
#define FOC_STAGE_CLOSED_LOOP       4U        /**< 闭环运行阶段 */
#define FOC_STAGE_FAULT             5U

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
/** @brief 定子电感离散项: Ls * Ibase / (Vbase * Ts) */
#define FOC_LS_OVER_TS_PU           (MOTOR_PHASE_INDUCTANCE * FOC_BASE_CURRENT_A \
                                     / (FOC_BASE_VOLTAGE_V * FOC_CTRL_TS))

/*--- 龙伯格观测器/PLL 诊断参数（只观测，不参与控制） ---*/
#define FOC_OBS_MAX_SPEED_RPM       3000.0f
#define FOC_OBS_MAX_OMEGA_ELEC      (FOC_OBS_MAX_SPEED_RPM * FOC_2PI \
                                     * (float)MOTOR_POLE_PAIRS / 60.0f)
#define FOC_OBS_LPF_GAIN            0.03125f
#define FOC_OBS_PLL_KP              0.08f
#define FOC_OBS_PLL_KI              0.002f
#define FOC_OBS_PLL_SPEED_BLEND     0.002f
#define FOC_OBS_PLL_ERR_MAX         0.125f
#define FOC_OBS_LOCK_BEMF_SQ_THR    0.0010f
#define FOC_OBS_LOCK_SPEED_RPM      100.0f
#define FOC_ELEC_OMEGA_TO_RPM       (60.0f / (FOC_2PI * (float)MOTOR_POLE_PAIRS))

/*--- PI 电流环增益（小电流电机降低增益防振荡） ---*/
#define FOC_PI_ID_KP                0.05f
#define FOC_PI_ID_KI                0.001f
#define FOC_PI_IQ_KP                0.05f
#define FOC_PI_IQ_KI                0.001f
/*--- PI 速度环增益（按参考工程 Q12：K_float = K_q12 / 4096） ---*/
#define FOC_PI_SPEED_KP             0.910f      /**< 2500/4096, 输出单位为Q12计数 */
#define FOC_PI_SPEED_KI             0.00044f    /**< 10/4096, 输出单位为Q12计数 */
#define FOC_CLOSED_IQ_REF_MAX_Q12   800.0f      /**< 闭环 IqRef 上限 (Q12计数, 约0.39A@2A基值) */
#define FOC_PI_SPEED_OUT_MAX        FOC_CLOSED_IQ_REF_MAX_Q12
#define FOC_PI_SPEED_OUT_MIN        (-FOC_CLOSED_IQ_REF_MAX_Q12)
#define FOC_CLOSED_IQ_REF_MAX       FOC_Q12_TO_PU(FOC_CLOSED_IQ_REF_MAX_Q12)
#define FOC_PI_SPEED_ERR_DEADBAND_RPM 2.0f    /**< 速度环误差死区，防止零点附近抖动 */
#define FOC_PI_SPEED_UNWIND_KP      0.05f     /**< 过速/欠速时反向积分软泄放系数 (Q12计数/RPM) */
#define FOC_PI_SPEED_SLEW_UP_Q12    12.0f     /**< 速度PI输出上升斜率限制 (Q12计数/速度环周期) */
#define FOC_PI_SPEED_SLEW_DOWN_Q12  8.0f      /**< 速度PI输出下降斜率限制 (Q12计数/速度环周期) */
#define FOC_SPEED_LOOP_DECIMATION   20U          /**< 速度环 10kHz/20=500Hz */
#define FOC_SPEED_RAMP_DIV          10U          /**< 每 10 个速度 PI 周期更新一次目标斜坡 */
#define FOC_SPEED_RAMP_STEP_RPM     2.0f         /**< 目标转速斜坡步长, 约 100rpm/s */

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
    FOC_MODE_IF_OPENLOOP = 1,   /**< IF 开环运行 */
    FOC_MODE_TRANSITION  = 2,   /**< 开环角度到观测角度渐变 */
    FOC_MODE_CLOSED_LOOP = 3,   /**< 速度闭环运行 */
    FOC_MODE_FAULT       = 4    /**< 故障保护 */
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
    uint32_t  u32SettleFrames;  /**< 达到切换转速后的恒速等待帧数 */
    uint32_t  u32SpeedStableCount; /**< 观测速度稳定连续计数 */
    uint32_t  u32BlendCount;    /**< 角度渐变计数器 */
    uint32_t  u32SwitchStableCount; /**< 闭环切换后稳定计数 */
    uint32_t  u32ObsLostCount;  /**< 观测器失锁计数 */
    float     f32ThetaErrSave;  /**< 过渡开始时 θobs - θif 的带符号误差 (rad) */
    float     f32ThetaIfRef;    /**< 过渡开始时 IF 角度 (rad) */
    float     f32TorqueAngle;   /**< 观测角度相对控制角度的正向角差 (rad) */
    float     f32RampedTargetRpm; /**< 速率限制后的目标转速 (RPM) */
    float     f32SpdIntegral;   /**< 速度 PI 积分项诊断 (pu) */
    float     f32SpdProportional; /**< 速度 PI 比例项诊断 (pu) */
    float     f32IqBase;        /**< 闭环维持 Iq 基准 (pu), 速度 PI 在此基础上追加 */
    float     f32IqBaseTarget;  /**< 闭环维持 Iq 基准目标 (pu) */
    uint16_t  u16DiagTransitionFlag; /**< 诊断：过渡阶段标志 */
    uint16_t  u16DiagStage;     /**< 诊断阶段码: FOC_STAGE_xxx */
} FOC_ControlState;

/**
  * @brief PI 控制器
  */
typedef struct
{
    float fKp;
    float fKi;
    float fErrPrev;
    float fOutPrev;
    float fIntegral;
    float fProportional;
    float fOutMax;
    float fOutMin;
} FOC_PI;

/**
  * @brief 龙伯格反电势观测器状态（旁路诊断，不参与开环控制）
  */
typedef struct
{
    float    f32Ealpha;       /**< α轴反电势估计 (pu) */
    float    f32Ebeta;        /**< β轴反电势估计 (pu) */
    float    f32ThetaObs;     /**< 观测电角度 (rad) */
    float    f32SpeedObs;     /**< 观测机械转速 (RPM) */
    float    f32ErrPll;       /**< PLL 误差 */
    float    f32BemfMag;      /**< BEMF 幅值近似 (pu) */
    uint16_t u16Locked;       /**< 锁定标志 */
} FOC_Luenberger;

/* Exported variables --------------------------------------------------------*/

extern FOC_MotorState    g_stMotor;
extern FOC_ControlState  g_stCtrl;
extern FOC_PI            g_stPiId;
extern FOC_PI            g_stPiIq;
extern FOC_PI            g_stPiSpeed;
extern FOC_Luenberger    g_stLuenberger;
extern float             FOC_fIaOffsetAdc;
extern float             FOC_fIbOffsetAdc;
extern const float       FOC_SinTable_F32[FOC_SIN_TABLE_SIZE];

/* Exported function prototypes ----------------------------------------------*/

void  FOC_Init(void);
void  FOC_ControlStep(void);
void  FOC_GetPhaseCurrent(void);
void  FOC_Luenberger_Init(void);
void  FOC_Luenberger_Run(void);

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
