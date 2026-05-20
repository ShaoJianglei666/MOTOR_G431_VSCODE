/**
  ******************************************************************************
  * @file    vf_ctrl.c
  * @brief   V/F（电压/频率）开环启动控制 — 实现
  *
  *          控制流程（每 10kHz 中断调用 VF_ControlStep）:
  *            1. 阶段状态机（STOP → ALIGN → RAMPING → RUNNING）
  *            2. 频率斜坡更新（恒定加速度）
  *            3. 电角度积分（θ += ω_elec * Ts）
  *            4. V/f 曲线计算 Vq（含低频提升补偿）
  *            5. 反 Park 变换 → SVPWM → 更新 TIM1 CCR
  *
  *          电流采样共用 FOC 模块的 FOC_GetPhaseCurrent()，仅用于监测保护。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "vf_ctrl.h"
#include "main.h"
#include <string.h>
#include <math.h>

/*============================================================================*/
/* 全局变量                                                                    */
/*============================================================================*/

VF_ControlState g_stVFCtrl;

/*============================================================================*/
/* 私有辅助函数                                                                */
/*============================================================================*/

/**
  * @brief 浮点数限幅
  */
static inline float vf_clamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/**
  * @brief 角度归一化到 [0, 2π)
  */
static inline float vf_wrap_2pi(float theta)
{
    while (theta >= VF_2PI) theta -= VF_2PI;
    while (theta < 0.0f)    theta += VF_2PI;
    return theta;
}

/**
  * @brief 正弦查找表 (256 点，与 FOC 共享)
  *        直接引用 FOC 模块的正弦表
  */
extern const float FOC_SinTable_F32[256];

#define VF_SIN_TABLE_SIZE       256U
#define VF_ANGLE_TO_IDX_SCALE   40.743665f   /* 256 / 2π */

static inline void vf_sincos(float theta, float *psin, float *pcos)
{
    uint8_t idx = ((uint32_t)(theta * VF_ANGLE_TO_IDX_SCALE)) & 0xFFU;
    *psin = FOC_SinTable_F32[idx];
    *pcos = FOC_SinTable_F32[((uint16_t)idx + 64U) & 0xFFU];
}

/*============================================================================*/
/* SVPWM — 共模注入法，零序分量注入                                            */
/*============================================================================*/
void VF_Svpwm(float fValpha, float fVbeta,
              uint16_t *pu16Ta, uint16_t *pu16Tb, uint16_t *pu16Tc)
{
    float fVa, fVb, fVc;
    float fVmin, fVmax, fVoffset;
    float fArr = 8500.0f;  /* TIM1 ARR/2 */

    /* 反 Clarke: 从 αβ 到三相 */
    fVa = fValpha;
    fVb = -0.5f * fValpha + 0.8660254037844386f * fVbeta;
    fVc = -0.5f * fValpha - 0.8660254037844386f * fVbeta;

    /* 共模注入（零序注入） */
    fVmin = fVa;
    if (fVb < fVmin) fVmin = fVb;
    if (fVc < fVmin) fVmin = fVc;
    fVmax = fVa;
    if (fVb > fVmax) fVmax = fVb;
    if (fVc > fVmax) fVmax = fVc;
    fVoffset = -0.5f * (fVmin + fVmax);

    fVa += fVoffset;
    fVb += fVoffset;
    fVc += fVoffset;

    /* 缩放至 [0, ARR] */
    *pu16Ta = (uint16_t)vf_clamp((fVa + 1.0f) * 0.5f * fArr, 0.0f, fArr);
    *pu16Tb = (uint16_t)vf_clamp((fVb + 1.0f) * 0.5f * fArr, 0.0f, fArr);
    *pu16Tc = (uint16_t)vf_clamp((fVc + 1.0f) * 0.5f * fArr, 0.0f, fArr);
}

/*============================================================================*/
/* 反 Park 变换                                                                */
/*============================================================================*/
void VF_InvPark(float fVd, float fVq, float fTheta,
                float *pfValpha, float *pfVbeta)
{
    float fSin, fCos;
    vf_sincos(fTheta, &fSin, &fCos);
    *pfValpha = fVd * fCos - fVq * fSin;
    *pfVbeta  = fVd * fSin + fVq * fCos;
}

/*============================================================================*/
/* V/f 曲线计算                                                               */
/*============================================================================*/

/**
  * @brief 根据当前转速（机械 RPM）计算 V/f 曲线输出电压 (pu)
  *
  *        曲线形状:
  *          f < f_corner:  V = (f/f_rated) * VF_VF_RATIO + V_boost
  *          f ≥ f_corner:  V = V_max（进入限压区）
  *
  *        其中 f 为电频率 (Hz)，f_rated = PolePairs * 1000 / 60
  *
  * @param  fRpm  当前机械转速 (RPM)
  * @return 电压幅值 (pu)
  */
static float VF_GetVoltagePu(float fRpm)
{
    float fFreqHz;         /* 电频率 (Hz) */
    float fFreqPu;         /* 频率标幺值 */
    float fVpu;            /* 输出电压 (pu) */

    /* 计算电频率 */
    fFreqHz = fRpm * (float)MOTOR_POLE_PAIRS / 60.0f;

    /* 频率标幺值（以额定频率为基值） */
    fFreqPu = fFreqHz / VF_RATED_FREQ_HZ;

    if (fFreqPu <= 0.0f)
    {
        fVpu = 0.0f;
    }
    else if (fFreqPu < VF_CORNER_FREQ_PU)
    {
        /* V/f 线性区: V = ratio * freq + boost */
        fVpu = VF_VF_RATIO * fFreqHz / VF_BASE_VOLTAGE_V + VF_BOOST_VOLTAGE_PU;
    }
    else
    {
        /* 恒压区（弱磁区）: 限幅至最大电压 */
        fVpu = VF_MAX_VOLTAGE_PU;
    }

    /* 保存诊断值 */
    g_stVFCtrl.f32FreqHz     = fFreqHz;
    g_stVFCtrl.f32VfOutputPu = fVpu;

    return vf_clamp(fVpu, 0.0f, VF_MAX_VOLTAGE_PU);
}

/*============================================================================*/
/* 电流采样 — 复用 ADC 数据（与 FOC 共用硬件）                                  */
/*============================================================================*/
void VF_GetPhaseCurrent(void)
{
    uint16_t u16IaRaw, u16IbRaw;
    float fIa, fIb;

    u16IaRaw = (uint16_t)(ADC1->JDR1);
    u16IbRaw = (uint16_t)(ADC2->JDR1);

    fIa = -1.0f * ((float)u16IaRaw - 1972.0f) / 2048.0f;
    fIb = -1.0f * ((float)u16IbRaw - 1972.0f) / 2048.0f;

    g_stVFCtrl.f32Ia = fIa;
    g_stVFCtrl.f32Ib = fIb;

    /* Clarke 变换得到 αβ 电流 */
    g_stVFCtrl.f32Ialpha = fIa;
    g_stVFCtrl.f32Ibeta  = (fIa + 2.0f * fIb) * VF_INV_SQRT3;

    /* 电流矢量幅值近似: max(|Ialpha|, |Ibeta|) + 0.5*min(|Ialpha|, |IBeta|) */
    {
        float fa = fIa;
        float fb = (fIa + 2.0f * fIb) * VF_INV_SQRT3;
        if (fa < 0.0f) fa = -fa;
        if (fb < 0.0f) fb = -fb;
        if (fa < fb)
        {
            float ft = fa; fa = fb; fb = ft;
        }
        g_stVFCtrl.f32CurrentMag = fa + 0.5f * fb;
    }

}

/*============================================================================*/
/* VF 控制初始化                                                              */
/*============================================================================*/
void VF_Init(void)
{
    memset(&g_stVFCtrl, 0, sizeof(g_stVFCtrl));

    g_stVFCtrl.eStage             = VF_STAGE_STOP;
    g_stVFCtrl.f32TargetRpm       = 0.0f;
    g_stVFCtrl.f32AccelRpmPerSec  = VF_ACCEL_RPM_PER_SEC;
    g_stVFCtrl.f32CurrentRpm      = 0.0f;
    g_stVFCtrl.f32Theta           = 0.0f;
    g_stVFCtrl.f32VdRef           = 0.0f;
    g_stVFCtrl.f32VqRef           = 0.0f;
    g_stVFCtrl.f32AlignVdRef      = 0.0f;
    g_stVFCtrl.u32SoftStartCount  = 0;
}

/*============================================================================*/
/* 设置目标转速                                                               */
/*============================================================================*/
void VF_SetTargetRpm(float fTargetRpm)
{
    g_stVFCtrl.f32TargetRpm = fTargetRpm;

    /* 如果当前停止且目标 > 0，自动进入启动流程 */
    if ((g_stVFCtrl.eStage == VF_STAGE_STOP) && (fTargetRpm > 0.0f))
    {
        g_stVFCtrl.eStage        = VF_STAGE_ALIGN;
        g_stVFCtrl.f32CurrentRpm = 0.0f;
        g_stVFCtrl.f32Theta      = 0.0f;
        g_stVFCtrl.u32AlignCount = 0;
        g_stVFCtrl.u32SoftStartCount = 0;
        g_stVFCtrl.u32RunFrames  = 0;
    }
}

/*============================================================================*/
/* 设置加速度                                                                 */
/*============================================================================*/
void VF_SetAccel(float fAccelRpmPerSec)
{
    if (fAccelRpmPerSec > 0.0f)
    {
        g_stVFCtrl.f32AccelRpmPerSec = fAccelRpmPerSec;
    }
}

/*============================================================================*/
/* 停止                                                                       */
/*============================================================================*/
void VF_Stop(void)
{
    g_stVFCtrl.eStage       = VF_STAGE_STOP;
    g_stVFCtrl.f32TargetRpm = 0.0f;
    g_stVFCtrl.f32CurrentRpm = 0.0f;
    g_stVFCtrl.f32Theta     = 0.0f;

    /* 输出 50% 占空比（安全状态） */
    TIM1->CCR1 = PWM_HALF_CYCLE_VF;
    TIM1->CCR2 = PWM_HALF_CYCLE_VF;
    TIM1->CCR3 = PWM_HALF_CYCLE_VF;
}

/*============================================================================*/
/* VF 主控制步进 — 10kHz 中断中调用                                           */
/*============================================================================*/
void VF_ControlStep(void)
{
    float fSpeedStep;     /* 每帧转速增量 (RPM) */
    float fOmegaElec;     /* 电角速度 (rad/s) */
    float fStepAngle;     /* 每帧角度增量 (rad) */
    float fVqPu;          /* q 轴电压指令 (pu) */

    /*======================================================================*/
    /* 阶段状态机                                                            */
    /*======================================================================*/
    switch (g_stVFCtrl.eStage)
    {
    /*----------------------------------------------------------------------*/
    /* STOP: 输出 50% 占空比，等待启动指令                                     */
    /*----------------------------------------------------------------------*/
    case VF_STAGE_STOP:
        TIM1->CCR1 = PWM_HALF_CYCLE_VF;
        TIM1->CCR2 = PWM_HALF_CYCLE_VF;
        TIM1->CCR3 = PWM_HALF_CYCLE_VF;
        return;
        /* break; — unreachable */

    /*----------------------------------------------------------------------*/
    /* ALIGN: 转子预对齐                                                     */
    /*        在 d 轴方向施加固定电压矢量（θ=0, Vd=小值, Vq=0），              */
    /*        产生静止磁场将转子拉至已知电角度 0 位置。                        */
    /*----------------------------------------------------------------------*/
    case VF_STAGE_ALIGN:
    {
        g_stVFCtrl.u32AlignCount++;

        if (g_stVFCtrl.u32AlignCount <= VF_ALIGN_FRAMES)
        {
            /*--- 对齐: 固定方向 d 轴电压 ---*/
            g_stVFCtrl.f32Theta      = 0.0f;
            g_stVFCtrl.f32VdRef      = VF_ALIGN_VOLTAGE_PU;
            g_stVFCtrl.f32VqRef      = 0.0f;
            g_stVFCtrl.f32AlignVdRef = VF_ALIGN_VOLTAGE_PU;
        }
        else
        {
            /*--- 对齐完成 → 进入软过渡 ---*/
            g_stVFCtrl.eStage           = VF_STAGE_RAMPING;
            g_stVFCtrl.u32SoftStartCount = 0;
            g_stVFCtrl.f32CurrentRpm    = VF_START_RPM;
            g_stVFCtrl.f32Theta         = 0.0f;
        }
        break;
    }

    /*----------------------------------------------------------------------*/
    /* RAMPING: 频率斜坡加速                                                  */
    /*          含 ALIGN→RAMPING 软过渡                                     */
    /*----------------------------------------------------------------------*/
    case VF_STAGE_RAMPING:
    {
        /*--- 计算每帧转速增量 ---*/
        fSpeedStep = g_stVFCtrl.f32AccelRpmPerSec * VF_CTRL_TS;

        /*--- 频率斜坡 ---*/
        g_stVFCtrl.f32CurrentRpm += fSpeedStep;

        if (g_stVFCtrl.f32CurrentRpm >= g_stVFCtrl.f32TargetRpm)
        {
            g_stVFCtrl.f32CurrentRpm = g_stVFCtrl.f32TargetRpm;
            g_stVFCtrl.eStage = VF_STAGE_RUNNING;
        }

        /*--- 电角度积分 ---*/
        fOmegaElec = g_stVFCtrl.f32CurrentRpm
                   * VF_2PI * (float)MOTOR_POLE_PAIRS / 60.0f;
        fStepAngle = fOmegaElec * VF_CTRL_TS;
        g_stVFCtrl.f32Theta = vf_wrap_2pi(g_stVFCtrl.f32Theta + fStepAngle);

        /*--- V/f 曲线计算 Vq ---*/
        fVqPu = VF_GetVoltagePu(g_stVFCtrl.f32CurrentRpm);
        g_stVFCtrl.f32VqRef = fVqPu;

        /*==================================================================*/
        /* ALIGN → RAMPING 软过渡                                           */
        /* 在软过渡期间，Vd 从对齐值线性衰减到 0，Vq 从 0 线性增长到目标值    */
        /* 避免电压矢量方向突变导致电机抖动                                   */
        /*==================================================================*/
        if (g_stVFCtrl.u32SoftStartCount < VF_SOFTSTART_FRAMES)
        {
            float fProgress = (float)g_stVFCtrl.u32SoftStartCount
                            / (float)VF_SOFTSTART_FRAMES;
            /* Vd 从对齐值线性衰减到 0 */
            g_stVFCtrl.f32VdRef = g_stVFCtrl.f32AlignVdRef * (1.0f - fProgress);
            /* Vq 从 0 线性增长到目标值 */
            g_stVFCtrl.f32VqRef = fVqPu * fProgress;
            g_stVFCtrl.u32SoftStartCount++;
        }
        else
        {
            /* 软过渡结束，纯 Vq 输出 */
            g_stVFCtrl.f32VdRef = 0.0f;
            g_stVFCtrl.f32VqRef = fVqPu;
        }
        break;
    }

    /*----------------------------------------------------------------------*/
    /* RUNNING: 到达目标转速，恒速运行                                         */
    /*----------------------------------------------------------------------*/
    case VF_STAGE_RUNNING:
    {
        g_stVFCtrl.f32CurrentRpm = g_stVFCtrl.f32TargetRpm;

        fOmegaElec = g_stVFCtrl.f32CurrentRpm
                   * VF_2PI * (float)MOTOR_POLE_PAIRS / 60.0f;
        fStepAngle = fOmegaElec * VF_CTRL_TS;
        g_stVFCtrl.f32Theta = vf_wrap_2pi(g_stVFCtrl.f32Theta + fStepAngle);

        fVqPu = VF_GetVoltagePu(g_stVFCtrl.f32CurrentRpm);
        g_stVFCtrl.f32VdRef = 0.0f;
        g_stVFCtrl.f32VqRef = fVqPu;
        break;
    }

    /*----------------------------------------------------------------------*/
    /* FAULT: 故障保护                                                        */
    /*----------------------------------------------------------------------*/
    case VF_STAGE_FAULT:
        TIM1->CCR1 = PWM_HALF_CYCLE_VF;
        TIM1->CCR2 = PWM_HALF_CYCLE_VF;
        TIM1->CCR3 = PWM_HALF_CYCLE_VF;
        return;
        /* break; — unreachable */
    }

    /*======================================================================*/
    /* 电流限制 — 如果电流幅值超过阈值，按比例降低 Vq                        */
    /*======================================================================*/
    if (g_stVFCtrl.f32CurrentMag > VF_CURRENT_LIMIT_PU)
    {
        float fScale = VF_CURRENT_LIMIT_PU / g_stVFCtrl.f32CurrentMag;
        g_stVFCtrl.f32VqRef *= fScale;
    }

    /*======================================================================*/
    /* 反 Park 变换: dq → αβ                                               */
    /*======================================================================*/
    VF_InvPark(g_stVFCtrl.f32VdRef, g_stVFCtrl.f32VqRef,
               g_stVFCtrl.f32Theta,
               &g_stVFCtrl.f32Valpha, &g_stVFCtrl.f32Vbeta);

    /*======================================================================*/
    /* SVPWM: αβ → 三相占空比                                                */
    /*======================================================================*/
    VF_Svpwm(g_stVFCtrl.f32Valpha, g_stVFCtrl.f32Vbeta,
             &g_stVFCtrl.u16Ta, &g_stVFCtrl.u16Tb, &g_stVFCtrl.u16Tc);

    /*======================================================================*/
    /* 更新 TIM1 比较寄存器                                                  */
    /*======================================================================*/
    TIM1->CCR1 = g_stVFCtrl.u16Ta;
    TIM1->CCR2 = g_stVFCtrl.u16Tb;
    TIM1->CCR3 = g_stVFCtrl.u16Tc;

    /*======================================================================*/
    /* 运行计数                                                              */
    /*======================================================================*/
    g_stVFCtrl.u32RunFrames++;
}
