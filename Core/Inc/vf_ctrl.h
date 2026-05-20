/**
  ******************************************************************************
  * @file    vf_ctrl.h
  * @brief   V/F（电压/频率）开环启动控制 — 独立模块
  *          不依赖电流 PI 闭环，通过 V/f 比直接输出电压矢量。
  *          适用于 PMSM/BLDC 电机开环起动至可观测反电势的转速。
  ******************************************************************************
  * @attention
  *   核心原理:
  *     1. 频率斜坡: 目标转速以恒定加速度从 0 升至设定值
  *     2. 角度生成: 电角度由频率积分得到（θ = ∫ ω·dt）
  *     3. V/f 曲线: Vq 随频率线性增长（含低频提升补偿定子电阻压降）
  *     4. 输出: Vd=0, Vq 由 V/f 曲线给出 → 反Park → SVPWM
  *
  *   电机参数来源于 Motor_Param.h（与 FOC 共用）
  ******************************************************************************
  */

#ifndef __VF_CTRL_H
#define __VF_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "Motor_Param.h"

/* Exported defines ---------------------------------------------------------*/

/**
  * @brief  USE_VF_CTRL — 控制模式全局选择
  *         = 1: 使用 V/F 开环启动（电压/频率比控制，无电流环）
  *         = 0: 使用原有 IF 开环（电流闭环 + 角度积分）
  *
  * 此宏供 stm32g4xx_it.c 和 main.c 共享。
  * 切换模式后请重新编译整个工程。
  */
#define USE_VF_CTRL                     1

/* Exported constants --------------------------------------------------------*/

/** @brief 控制频率 (Hz) — 与 FOC 共用 10kHz */
#define VF_CTRL_FREQ                    10000U
#define VF_CTRL_TS                      (1.0f / (float)VF_CTRL_FREQ)

/** @brief 圆周率 */
#define VF_MATH_PI                      3.14159265358979323846f
#define VF_2PI                          (2.0f * VF_MATH_PI)
#define VF_SQRT3                        1.7320508075688772f
#define VF_INV_SQRT3                    0.5773502691896257f

/** @brief 电压基值 (V) = Vbus / √3 */
#define VF_BASE_VOLTAGE_V               (MOTOR_BUS_VOLTAGE / VF_SQRT3)

/**
  * @brief V/f 比率 (V/Hz)
  *        按反电势系数计算额定 V/f：
  *        Ke = MOTOR_BEMF_CONST_V_LL V_peak L-L / krpm（机械）
  *        每相电压峰值 at 1000RPM = Ke / √3
  *        电频率 at 1000RPM = 1000 * PolePairs / 60
  *        V/f_ratio = 相电压峰值 / 电频率
  *
  *        ★ 注意：BEMF V/f 仅抵消反电势。要产生电流还需额外电压克服 IR 压降。
  *          因此实际 V/f 应略高于 BEMF V/f，这里取 105% 留足转矩余量。
  */
#define VF_RATED_FREQ_HZ                (1000.0f * (float)MOTOR_POLE_PAIRS / 60.0f)  /* 1000RPM 对应电频率 */
#define VF_NOMINAL_RATIO                ((MOTOR_BEMF_CONST_V_LL / VF_SQRT3) / VF_RATED_FREQ_HZ)
#define VF_VF_RATIO                     (VF_NOMINAL_RATIO * 1.05f)  /* 105% BEMF V/f，留足 IR 压降余量 */

/**
  * @brief 低频电压提升（IR 补偿）
  *        在低频时定子电阻压降占主导，需额外提升电压。
  *        此电机 R=0.006Ω、L=35µH，极低阻抗导致低频电流对电压极敏感。
  *        提升电压帮助克服静摩擦和齿槽转矩。
  *        取 10% 基值电压 ≈ 3.46V，约对应 3.46/0.006 ≈ 577A 峰值 — 实际受
  *        电源限流和 MOSFET 导通电阻限制，不会达到此值。
  *        如仍顿挫可逐步增大此值。
  */
#define VF_BOOST_VOLTAGE_PU             0.10f

/** @brief 最大输出电压 (pu)，限制 SVPWM 不过调制 */
#define VF_MAX_VOLTAGE_PU               0.92f

/**
  * @brief V/f 拐点频率 (pu of rated freq)
  *        低于此频率 V/f 为线性，高于此频率电压保持恒定（弱磁区）
  *        默认 1.0 = 额定频率
  */
#define VF_CORNER_FREQ_PU               1.0f

/*--- 频率斜坡参数 ---*/

/** @brief 加速度 (RPM/s) — 降低加速度防止失步 */
#define VF_ACCEL_RPM_PER_SEC            200.0f

/** @brief 每帧转速增量 (RPM) = Accel * Ts */
#define VF_SPEED_STEP_RPM               (VF_ACCEL_RPM_PER_SEC * VF_CTRL_TS)

/** @brief 起始频率 (RPM) — 从该转速开始施加电压
  *        提高起始转速，让电机更快跳过齿槽转矩影响严重的极低速区
  */
#define VF_START_RPM                    80.0f

/** @brief 启动前对齐时间 (帧) — 施加直流电压对齐转子 */
#define VF_ALIGN_FRAMES                 1500U   /* 150ms @ 10kHz */

/** @brief 对齐电压 (pu) — 对齐阶段 d 轴电压幅值 */
#define VF_ALIGN_VOLTAGE_PU             0.04f

/** @brief ALIGN→RAMPING 软过渡帧数 — 从 Vd 对齐平滑切到 Vq 旋转 */
#define VF_SOFTSTART_FRAMES             1000U   /* 100ms @ 10kHz */

/** @brief 电流限制 (pu) — 超过此值则降低 Vq 防止过流
  *        基于 ADC 采样的相电流幅值做简单限幅
  */
#define VF_CURRENT_LIMIT_PU             0.50f   /* 5A@10A基值 */

/** @brief VF→IF 切换延时 (帧) — VF_RUNNING 后等待 2s 再切 */
#define VF_IF_SWITCH_DELAY              20000U  /* 2s @ 10kHz */

/** @brief VF→IF blend 过渡帧数 */
#define VF_IF_BLEND_FRAMES              20000U  /* 2s @ 10kHz */

/** @brief IF 电流目标 (pu) — αβ 电流幅值 */
#define VF_IF_I_TARGET_PU               0.15f   /* 1.5A */

/** @brief IF PI 增益（单 PI 控 αβ 幅值） */
#define VF_IF_PI_KP                     0.02f
#define VF_IF_PI_KI                     0.0002f

/** @brief IF PI 输出限幅 (pu) */
#define VF_IF_PI_OUT_MAX                0.85f
#define VF_IF_PI_OUT_MIN                (-VF_IF_PI_OUT_MAX)

/** @brief IF 过流保护阈值 (pu) — 暂提高避免切换毛刺误触发 */
#define VF_IF_OC_LIMIT_PU               1.5f    /* 15A */

/** @brief 对齐占空比 (50%=中点) */
#define PWM_HALF_CYCLE_VF               (17000U / 4U)

/* Exported types ------------------------------------------------------------*/

/**
  * @brief VF 控制阶段
  */
typedef enum
{
    VF_STAGE_STOP      = 0,  /**< 停止 */
    VF_STAGE_ALIGN     = 1,  /**< 转子预对齐 */
    VF_STAGE_RAMPING   = 2,  /**< 频率斜坡加速 */
    VF_STAGE_RUNNING   = 3,  /**< VF 恒速运行 */
    VF_STAGE_IF_BLEND  = 4,  /**< VF→IF 过渡 blend */
    VF_STAGE_IF_RUNNING= 5,  /**< IF 电流环闭环运行 */
    VF_STAGE_FAULT     = 6   /**< 故障 */
} VF_Stage;

/**
  * @brief PI 控制器（增量式）
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
} VF_PI;

/**
  * @brief VF 控制状态结构体
  */
typedef struct
{
    /*--- 用户设置 ---*/
    float     f32TargetRpm;     /**< 目标转速 (RPM) */
    float     f32AccelRpmPerSec;/**< 加速度 (RPM/s) */

    /*--- 运行状态 ---*/
    VF_Stage  eStage;           /**< 当前阶段 */
    float     f32CurrentRpm;    /**< 当前频率对应转速 (RPM) */
    float     f32Theta;         /**< 当前电角度 (rad) [0, 2π) */
    float     f32VdRef;         /**< d 轴电压指令 (pu) */
    float     f32VqRef;         /**< q 轴电压指令 (pu) */
    float     f32Valpha;        /**< α 轴电压 (pu) */
    float     f32Vbeta;         /**< β 轴电压 (pu) */
    uint16_t  u16Ta;            /**< A 相 PWM 比较值 */
    uint16_t  u16Tb;            /**< B 相 PWM 比较值 */
    uint16_t  u16Tc;            /**< C 相 PWM 比较值 */
    uint32_t  u32AlignCount;    /**< 对齐计数器 */
    uint32_t  u32RunFrames;     /**< 总运行帧数 */

    /*--- 软启动状态 ---*/
    uint32_t  u32SoftStartCount;/**< 软过渡计数器 */
    float     f32AlignVdRef;    /**< 对齐时的 Vd 值，软过渡用 */

    /*--- VF→IF 切换状态 ---*/
    uint32_t  u32VfRunCount;    /**< VF_RUNNING 运行帧数 */
    uint32_t  u32IfBlendCount;  /**< IF blend 计数器 */
    float     f32IfBlendVqStart;/**< blend 起始 Vq (V/f 值) */

    /*--- IF 电流环状态（单 PI 控 αβ 电流幅值，不依赖 Park）---*/
    float     f32IfITarget;     /**< αβ 电流幅值目标 (pu) */
    VF_PI     stPiMag;          /**< αβ 电流幅值 PI */
    float     f32PiMagOut;      /**< PI 输出 (pu) */

    /*--- 电流监控 ---*/
    float     f32Ia;            /**< A 相电流 (pu) */
    float     f32Ib;            /**< B 相电流 (pu) */
    float     f32Ialpha;        /**< α 轴电流 (pu) */
    float     f32Ibeta;         /**< β 轴电流 (pu) */
    float     f32CurrentMag;    /**< 电流空间矢量幅值 (pu) */

    /*--- 诊断 ---*/
    float     f32VfOutputPu;    /**< V/f 曲线输出电压 (pu) */
    float     f32FreqHz;        /**< 当前电频率 (Hz) */
} VF_ControlState;

/* Exported variables --------------------------------------------------------*/

extern VF_ControlState g_stVFCtrl;

/* Exported function prototypes ----------------------------------------------*/

void  VF_Init(void);
void  VF_SetTargetRpm(float fTargetRpm);
void  VF_SetAccel(float fAccelRpmPerSec);
void  VF_Stop(void);
void  VF_ControlStep(void);
void  VF_Svpwm(float fValpha, float fVbeta,
               uint16_t *pu16Ta, uint16_t *pu16Tb, uint16_t *pu16Tc);
void  VF_InvPark(float fVd, float fVq, float fTheta,
                 float *pfValpha, float *pfVbeta);
void  VF_GetPhaseCurrent(void);

#ifdef __cplusplus
}
#endif

#endif /* __VF_CTRL_H */
