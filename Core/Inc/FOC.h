/**
  ******************************************************************************
  * @file    FOC.h
  * @brief   FOC（磁场定向控制）算法库头文件
  *          包含 SVPWM、Clarke、Park 等变换的函数声明及常量定义。
  *          【v2.0】已升级为浮点运算版本，适用于带 FPU 的 MCU（如 STM32G4）。
  ******************************************************************************
  * @attention
  *
  * 本模块所有物理量均采用浮点数（float）表示：
  *   - 角度 θ:   浮点弧度 [0, 2π)
  *   - 电压 V:   标幺值 [-1.0, 1.0] pu（1.0 pu = Vdc/√3）
  *   - 电流 I:   标幺值 [-1.0, 1.0] pu（1.0 pu = FOC_BASE_CURRENT_A）
  *   - 转速:     float RPM
  *   - PWM 周期: 由 FOC_PWM_PERIOD_CYCLES 定义，对应 TIM 的 ARR 值
  *
  * 使用前需根据实际 PWM 定时器配置调整 FOC_PWM_PERIOD_CYCLES 的值。
  *
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

/**
  * @brief PWM 定时器有效周期（中心对齐模式下的计数值 ×2）
  *        当前 TIM1 配置：中心对齐模式，ARR=8500
  *        等效全周期 = ARR × 2 = 17000
  */
#define FOC_PWM_PERIOD_CYCLES       17000U

/** @brief PWM 半周期（中心对齐模式下 ARR 值） */
#define FOC_PWM_ARR                 (FOC_PWM_PERIOD_CYCLES / 2U)

/** @brief 50% 占空比基准点（中心对齐模式） */
#define PWM_HALF_CYCLE              (FOC_PWM_PERIOD_CYCLES / 4U)

/** @brief √3 */
#define FOC_SQRT3                   1.7320508075688772f
/** @brief 1/√3 */
#define FOC_INV_SQRT3               0.5773502691896257f

/* Exported types ------------------------------------------------------------*/

/**
  * @brief FOC 电机电气状态结构体
  *        保存电机运行时测量的电气量，仅含原始数据和变换结果，
  *        不包含参考值和控制状态。
  */
typedef struct
{
    /*--- 角度 ---*/
    float f32Theta;          /**< 当前电角度 (rad, [0, 2π)) */

    /*--- 电压量 (pu) ---*/
    float f32UdRef;          /**< d轴电压指令 (pu) */
    float f32UqRef;          /**< q轴电压指令 (pu) */
    float f32Valpha;         /**< α轴电压 (pu) */
    float f32Vbeta;          /**< β轴电压 (pu) */

    /*--- PWM 输出 ---*/
    uint16_t u16Ta;          /**< A相PWM比较值 */
    uint16_t u16Tb;          /**< B相PWM比较值 */
    uint16_t u16Tc;          /**< C相PWM比较值 */
    uint16_t u16Sector;      /**< 当前扇区 (1~6) */

    /*--- 电流 (pu) ---*/
    float f32Ia;             /**< A相电流 (pu) */
    float f32Ib;             /**< B相电流 (pu) */
    float f32Ic;             /**< C相电流, Ic = -(Ia+Ib) (pu) */
    float f32Ialpha;         /**< α轴电流 (pu) */
    float f32Ibeta;          /**< β轴电流 (pu) */
    float f32Id;             /**< d轴电流 (pu) */
    float f32Iq;             /**< q轴电流 (pu) */
} FOC_MotorState;

/** @brief FOC 全局电机电气状态 */
extern FOC_MotorState g_stMotor;

/** @brief 电流零点 ADC 读数（偏移校准值，ADC 原始计数值） */
#define FOC_IA_OFFSET_ADC           1972.0f
#define FOC_IB_OFFSET_ADC           1972.0f

/** @brief 电流标度：ADC 每单位 pu 对应的计数值 (1.0pu = 2048 counts) */
#define FOC_ADC_CURRENT_SCALE       2048.0f

extern float FOC_fIaOffsetAdc;
extern float FOC_fIbOffsetAdc;

/*----------------------------------------------------------------------------*/
/* IF 开环调试模式开关                                                        */
/*   定义 FOC_IF_OPENLOOP_DEBUG 后：                                           */
/*     - 状态机停留在 IF_STARTUP 阶段，不切换到闭环                            */
/*     - 角度由开环斜坡积分生成（不依赖观测器）                                */
/*     - 电流环 PI 正常闭环，Iq=固定值，Id=0                                  */
/*     - 观测器关闭（节省 CPU，避免未收敛导致的诊断干扰）                      */
/*   取消定义则恢复完整的 Imperix 五阶段切换闭环逻辑。                         */
/*----------------------------------------------------------------------------*/
/* #define FOC_IF_OPENLOOP_DEBUG */

/**
  * @brief FOC 控制模式枚举（Imperix 五阶段切换法）
  */
typedef enum
{
    FOC_MODE_IDLE        = 0,  /**< 空闲/停止 */
    FOC_MODE_IF_STARTUP  = 1,  /**< 阶段2: IF开环加速（Iq固定，ω斜坡上升） */
    FOC_MODE_IF_ALIGN    = 2,  /**< 阶段3: 恒速对齐（ω恒定，Iq减速，θ_err收敛） */
    FOC_MODE_TRANSITION  = 3,  /**< 阶段4: 切换瞬间（θ→θ̂，速度PI预载） */
    FOC_MODE_CLOSED_LOOP = 4,  /**< 阶段5: 速度闭环（θ=θ̂，速度PI+电流PI） */
    FOC_MODE_FAULT       = 5   /**< 故障保护 */
} FOC_ControlMode;

/**
  * @brief FOC 控制状态机结构体（Imperix 五阶段切换法）
  */
typedef struct
{
    /*--- 当前状态 ---*/
    FOC_ControlMode eMode;
    FOC_ControlMode eModePrev;

    /*--- 参考值 ---*/
    float    f32TargetRpm;          /**< 目标转速 (RPM) */
    float    f32IdRef;             /**< d轴电流指令 (pu) */
    float    f32IqRef;             /**< q轴电流指令 (pu) */

    /*--- IF 开环加速参数 ---*/
    float    f32RpmRamp;           /**< 开环加速当前转速 (RPM) */

    /*--- 恒速对齐参数 ---*/
    uint16_t u16AlignCount;        /**< 对齐计数器（帧） */
    float    f32IqRampDown;        /**< Iq 连续下降量（pu） */
    uint16_t u16AlignFrames;       /**< 对齐已耗时（帧） */
    uint16_t u16SpeedStableCount;  /**< 速度稳定连续计数 */

    /*--- 切换参数 ---*/
    uint16_t u16SwitchStableCount; /**< 切换后稳定计数（帧） */
    float    f32ThetaIfRef;        /**< IF 参考角度（过渡渐变用, rad） */
    uint16_t u16BlendCount;        /**< 渐变计数器 */
    float    f32ThetaErrSave;      /**< 过渡开始时的角度差（rad） */

    /*--- 失锁检测 ---*/
    uint16_t u16ObsLostCount;

    float    f32TorqueAngle;       /**< 转矩角 (rad) */
    uint16_t u16DiagTransitionFlag; /**< 诊断：过渡标志 (1=TRANSITION中, 0=其他) */
} FOC_ControlState;

/** @brief FOC 全局控制状态机实例 */
extern FOC_ControlState g_stCtrl;

/* Exported constants --------------------------------------------------------*/

/*--- 切换参数 ---*/
#define FOC_CTRL_RPM_SWITCH         600.0f   /**< 切换转速 (RPM) */
#define FOC_CTRL_ALIGN_DURATION     1000U    /**< 初始对齐时长 (1000帧@10kHz=100ms) */
#define FOC_CTRL_IQ_ALIGN_START     0.1526f  /**< 对齐起始电流 (pu, =5000/32768) */
#define FOC_CTRL_RPM_ACCEL_STEP     50.0f    /**< 加速每步 RPM 增量 */
#define FOC_CTRL_RPM_ACCEL_INTERVAL 800U     /**< 加速间隔帧数 (800帧@10kHz=80ms) */
#define FOC_CTRL_IQ_STARTUP         0.0916f  /**< IF 启动电流 (pu, =3000/32768) */
#define FOC_CTRL_TORQUE_ANGLE_60    (FOC_MATH_PI / 3.0f)  /**< 60° 转矩角 (rad) */
#define FOC_CTRL_RPM_SETTLE_FRAMES  20000U   /**< IF 稳速等待帧数 (20000帧@10kHz=2秒) */
#define FOC_CTRL_SPEED_TOL_PCT      10U      /**< 转速容差百分比 (±10%) */
#define FOC_CTRL_SPEED_LOOP_DECIMATION 10U   /**< 速度环分频系数 (10kHz/10=1kHz) */
#define FOC_CTRL_STABLE_COUNT_THR   500U     /**< 速度稳定判定连续帧数 (500帧=50ms) */
#define FOC_CTRL_TRANSITION_FRAMES  5000U    /**< 角度渐进帧数 (5000帧=500ms) */
#define FOC_CTRL_STABLE_FRAMES      20000U   /**< 切换后稳定帧数 (20000帧@10kHz=2秒) */
#define FOC_CTRL_OBS_LOST_THRESHOLD 5000U    /**< 观测器失锁判定帧数 (500ms) */

/**
  * @brief PI 控制器结构体
  */
typedef struct
{
    float fKp;            /**< 比例增益 */
    float fKi;            /**< 积分增益 (已含 Ts) */
    float fIntegral;      /**< 积分累加器 */
    float fProportional;  /**< 比例项输出, 由 FOC_PI_Run 更新 */
    float fOutMax;        /**< 输出上限 */
    float fOutMin;        /**< 输出下限 */
} FOC_PI;

/** @brief PI 控制器默认增益（由 Q15 转换：K_float = K_Q15 / 32768） */
#define FOC_PI_ID_KP          0.25f     /* 原 8192/32768 */
#define FOC_PI_ID_KI          0.05f     /* 原 1638/32768 */
#define FOC_PI_IQ_KP          0.25f     /* 原 8192/32768 */
#define FOC_PI_IQ_KI          0.05f     /* 原 1638/32768 */

/** @brief PI 控制器实例 */
extern FOC_PI g_stPiId;
extern FOC_PI g_stPiIq;
extern FOC_PI g_stPiSpeed;

/** @brief 速度 PI 默认增益 */
#define FOC_PI_SPEED_KP       0.30518f  /* 原 10000/32768, Kp≈0.305, RPM→pu(Iq) */
#define FOC_PI_SPEED_KI       0.003906f /* 原 128/32768,   Ki≈0.0039 */
#define FOC_PI_SPEED_OUT_MAX  0.9155f   /* 原 30000/32768, Iq 上限 */
#define FOC_PI_SPEED_OUT_MIN (-0.9155f) /* Iq 下限 */

/**
  * @brief 龙伯格观测器状态结构体
  *        基于电机电压方程，在 αβ 静止坐标系下估计反电势，
  *        从而提取转子角度和速度。
  *
  *        输入：Vα, Vβ, Iα, Iβ（均为 pu 标幺值）
  *        输出：f32ThetaObs（电角度 rad）, f32SpeedObs（机械转速 RPM）
  */
typedef struct
{
    /*--- 观测器内部状态 ---*/
    float f32IalphaEst;      /**< 估计α轴电流 (pu) */
    float f32IbetaEst;       /**< 估计β轴电流 (pu) */
    float f32Ealpha;         /**< 估计α轴反电势 (pu) */
    float f32Ebeta;          /**< 估计β轴反电势 (pu) */

    /*--- 观测器输出 ---*/
    float f32SpeedObs;       /**< 估计机械转速 (RPM) */
    float f32ThetaObs;       /**< 估计电角度 (rad, [0, 2π)) */
    uint16_t u16Locked;      /**< 观测器锁定标志: 1=已收敛, 0=未收敛 */

    /*--- 诊断变量 ---*/
    float f32ErrPll;         /**< PLL 误差 (pu) */
    float f32BemfMag;        /**< BEMF 幅值 √(eα²+eβ²) (pu) */
} FOC_Luenberger;

/** @brief 龙伯格观测器实例 */
extern FOC_Luenberger g_stLuenberger;

/**
  * @brief VF 控制器状态结构体（低层级接口使用）
  */
typedef struct
{
    float f32Theta;          /**< 当前电角度 (rad) */
} FOC_VF_State;

/* Exported constants --------------------------------------------------------*/

/** @brief 最大电压标幺值 (pu) */
#define FOC_VOLTAGE_MAX_PU          1.0f

/** @brief 1.0 pu 对应的实际电压 (V) = Vdc / √3 */
#define FOC_VOLTAGE_1PU_V           (MOTOR_BUS_VOLTAGE / FOC_SQRT3)

/**
  * @brief V/F 斜率 (pu/RPM)
  *        由反电势系数推算：BEMF_phase_peak@1000rpm / 1pu_voltage / 1000
  */
#define FOC_VF_SLOPE                (((MOTOR_BEMF_CONST_V_LL / 2.0f) / FOC_VOLTAGE_1PU_V) / 1000.0f)

/**
  * @brief V/F 低速电压提升 (pu)，补偿定子电阻压降
  *        约为最大电压的 5%
  */
#define FOC_VF_BOOST                0.05f

/**
  * @brief FOC 控制环更新频率 (Hz)
  *        与 PWM 同频，由 TIM1 更新中断驱动
  */
#define FOC_CTRL_FREQ               10000U

/**
  * @brief FOC 控制周期 (秒)
  */
#define FOC_CTRL_TS                 (1.0f / (float)FOC_CTRL_FREQ)

/**
  * @brief PWM 开关频率 (Hz)
  */
#define FOC_PWM_FREQ                10000U

/**
  * @brief 通过正弦表偏移获取余弦值
  *        余弦相位超前正弦 90°，256点表中 90° = 64 个点
  * @param  idx  正弦表索引 (0~255)
  * @retval 浮点余弦值 [-1.0, 1.0]
  */
#define FOC_Cos(idx)  FOC_SinTable_F32[((uint16_t)(idx) + 64U) & 0xFFU]

/* Exported constants --------------------------------------------------------*/

/** @brief 正弦查找表大小 */
#define FOC_SIN_TABLE_SIZE          256U

/** @brief 角度→正弦表索引缩放因子 (256 / 2π ≈ 40.743665) */
#define FOC_ANGLE_TO_IDX_SCALE      40.743665f

/*--- 龙伯格观测器常数 (基于电机参数) ---*/

/** @brief 标幺化基值 */
#define FOC_BASE_VOLTAGE_V          (MOTOR_BUS_VOLTAGE / FOC_SQRT3)  /* ≈ 13.86V */
#define FOC_BASE_CURRENT_A          10.0f                             /* 10A */
#define FOC_BASE_OMEGA_RADS         314.159f                          /* 2π×50 */

/** @brief 定子电阻标幺值: Rs_pu = Rs×Ib/Vb */
#define FOC_RS_PU                   (MOTOR_PHASE_RESISTANCE * FOC_BASE_CURRENT_A / FOC_BASE_VOLTAGE_V)

/** @brief 定子电感标幺值: Ls_pu = Ls×Ib×ωb/Vb */
#define FOC_LS_PU                   (MOTOR_PHASE_INDUCTANCE * FOC_BASE_CURRENT_A * FOC_BASE_OMEGA_RADS / FOC_BASE_VOLTAGE_V)

/** @brief 永磁磁链标幺值: ψf_pu = Ke_phase_peak × ωb / (1000×Vb) */
#define FOC_PSIF_PU                 ((MOTOR_BEMF_CONST_V_LL / 2.0f) / 1000.0f * FOC_BASE_OMEGA_RADS / FOC_BASE_VOLTAGE_V)

/** @brief 离散时间步长 Ts_pu = Ts × ωb */
#define FOC_TS_PU                   (0.0001f * FOC_BASE_OMEGA_RADS)

/** @brief 观测器电流反馈增益 K1_disc = Ts_pu/Ls_pu */
#define FOC_OBS_K1_DISC             (FOC_TS_PU / FOC_LS_PU)

/** @brief 观测器低通滤波器系数 G1 (越小越平滑) */
#define FOC_OBS_G1                  0.03125f  /* 原 1024/32768 */

/** @brief 观测器反电势收敛增益 G2 */
#define FOC_OBS_G2                  0.0078125f /* 原 256/32768 */

/** @brief 电感补偿系数 Ls_comp (Q15→float: 688/32768) */
#define FOC_LS_COMP                 0.021f

/** @brief 反电势幅值平方锁定阈值 (pu², ≈(2700/32768)²≈0.0068) */
#define EMF_LOCK_SQ_THR_F           0.0065f

/** @brief 锁定最低转速阈值 (RPM) */
#define EMF_LOCK_SPD_THR_F          200.0f

/*--- PLL 增益 ---*/
/** @brief PLL 比例增益 (原 1024/32768) */
#define FOC_PLL_KP                  0.03125f
/** @brief PLL 积分增益 (原 512/32768) */
#define FOC_PLL_KI                  0.015625f
/** @brief PLL 误差限幅 (pu) */
#define FOC_PLL_ERR_MAX             0.125f   /* 原 4096/32768 */
/** @brief PLL 积分上限 (电气 rad/s, 对应约 10000RPM) */
#define FOC_PLL_OMEGA_MAX           1047.2f  /* ≈ 10000RPM × 4π/60 / 2 */

/** @brief 电气角速度 → 机械转速转换系数:
  *        RPM = ω_elec(rad/s) × 60 / (2π × pole_pairs) */
#define FOC_ELEC_OMEGA_TO_RPM       (60.0f / (FOC_2PI * (float)MOTOR_POLE_PAIRS))

/** @brief 正弦查找表（浮点格式，-1.0 ~ 1.0）
  *        表地址：sin_table[k] = sin(2πk/256)
  *        通过 FOC_Cos(idx) 宏获取余弦值
  */
extern const float FOC_SinTable_F32[FOC_SIN_TABLE_SIZE];

/* Exported function prototypes ----------------------------------------------*/

void FOC_Svpwm(float fValpha, float fVbeta,
               uint16_t *pu16Ta, uint16_t *pu16Tb,
               uint16_t *pu16Tc, uint16_t *pu16Sector);

void FOC_Clarke(float fIa, float fIb,
                float *pfIalpha, float *pfIbeta);

void FOC_Park(float fIalpha, float fIbeta, float fTheta,
              float *pfId, float *pfIq);

void FOC_InvPark(float fVd, float fVq, float fTheta,
                 float *pfValpha, float *pfVbeta);

void FOC_VF_Init(FOC_VF_State *pstVf);
void FOC_VF_Run(FOC_VF_State *pstVf, float fTargetRpm,
                float *pfUd, float *pfUq);

void FOC_Init(void);
float FOC_PI_Run(FOC_PI *pstPi, float fRef, float fFb);

void FOC_ControlStep(void);
void FOC_StateMachine_Init(void);
void FOC_StateMachine_Run(void);

void FOC_GetPhaseCurrent(void);

void FOC_Luenberger_Init(void);
void FOC_Luenberger_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H */
