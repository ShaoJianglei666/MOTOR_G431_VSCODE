/**
  ******************************************************************************
  * @file    FOC.c
  * @brief   FOC（磁场定向控制）算法库实现文件 【v2.0 浮点版本】
  *          适用于带 FPU 的 MCU（如 STM32G431），所有运算使用 float。
  *
  *          【SVPWM 算法说明】
  *          采用共模注入法（Common-Mode Injection）实现 SVPWM：
  *          1. 将 Vα, Vβ 转换为三相参考电压 Va, Vb, Vc
  *          2. 计算共模偏移 Voffset = -(Vmin + Vmax) / 2
  *          3. 注入共模分量后缩放至 PWM 比较值
  *          4. xyz 法判断扇区（与旧版兼容）
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FOC.h"
#include <string.h>
#include "main.h"

/* Private variables ---------------------------------------------------------*/

/**
  * @brief 浮点格式正弦查找表 (256点)
  *        sin_table[k] = sin(2πk/256), 范围 [-1.0, 1.0]
  *        通过 FOC_Cos(idx) 宏获取余弦值
  */
const float FOC_SinTable_F32[FOC_SIN_TABLE_SIZE] =
{
     0.000000f,  0.024541f,  0.049068f,  0.073565f,  0.098017f,  0.122411f,  0.146730f,  0.170962f,
     0.195090f,  0.219101f,  0.242980f,  0.266713f,  0.290285f,  0.313682f,  0.336890f,  0.359895f,
     0.382683f,  0.405241f,  0.427555f,  0.449611f,  0.471397f,  0.492898f,  0.514103f,  0.534998f,
     0.555570f,  0.575808f,  0.595699f,  0.615232f,  0.634393f,  0.653173f,  0.671559f,  0.689541f,
     0.707107f,  0.724247f,  0.740951f,  0.757209f,  0.773010f,  0.788346f,  0.803208f,  0.817585f,
     0.831470f,  0.844854f,  0.857729f,  0.870087f,  0.881921f,  0.893224f,  0.903989f,  0.914210f,
     0.923880f,  0.932993f,  0.941544f,  0.949528f,  0.956940f,  0.963776f,  0.970031f,  0.975702f,
     0.980785f,  0.985278f,  0.989177f,  0.992480f,  0.995185f,  0.997290f,  0.998795f,  0.999699f,
     1.000000f,  0.999699f,  0.998795f,  0.997290f,  0.995185f,  0.992480f,  0.989177f,  0.985278f,
     0.980785f,  0.975702f,  0.970031f,  0.963776f,  0.956940f,  0.949528f,  0.941544f,  0.932993f,
     0.923880f,  0.914210f,  0.903989f,  0.893224f,  0.881921f,  0.870087f,  0.857729f,  0.844854f,
     0.831470f,  0.817585f,  0.803208f,  0.788346f,  0.773010f,  0.757209f,  0.740951f,  0.724247f,
     0.707107f,  0.689541f,  0.671559f,  0.653173f,  0.634393f,  0.615232f,  0.595699f,  0.575808f,
     0.555570f,  0.534998f,  0.514103f,  0.492898f,  0.471397f,  0.449611f,  0.427555f,  0.405241f,
     0.382683f,  0.359895f,  0.336890f,  0.313682f,  0.290285f,  0.266713f,  0.242980f,  0.219101f,
     0.195090f,  0.170962f,  0.146730f,  0.122411f,  0.098017f,  0.073565f,  0.049068f,  0.024541f,
     0.000000f, -0.024541f, -0.049068f, -0.073565f, -0.098017f, -0.122411f, -0.146730f, -0.170962f,
    -0.195090f, -0.219101f, -0.242980f, -0.266713f, -0.290285f, -0.313682f, -0.336890f, -0.359895f,
    -0.382683f, -0.405241f, -0.427555f, -0.449611f, -0.471397f, -0.492898f, -0.514103f, -0.534998f,
    -0.555570f, -0.575808f, -0.595699f, -0.615232f, -0.634393f, -0.653173f, -0.671559f, -0.689541f,
    -0.707107f, -0.724247f, -0.740951f, -0.757209f, -0.773010f, -0.788346f, -0.803208f, -0.817585f,
    -0.831470f, -0.844854f, -0.857729f, -0.870087f, -0.881921f, -0.893224f, -0.903989f, -0.914210f,
    -0.923880f, -0.932993f, -0.941544f, -0.949528f, -0.956940f, -0.963776f, -0.970031f, -0.975702f,
    -0.980785f, -0.985278f, -0.989177f, -0.992480f, -0.995185f, -0.997290f, -0.998795f, -0.999699f,
    -1.000000f, -0.999699f, -0.998795f, -0.997290f, -0.995185f, -0.992480f, -0.989177f, -0.985278f,
    -0.980785f, -0.975702f, -0.970031f, -0.963776f, -0.956940f, -0.949528f, -0.941544f, -0.932993f,
    -0.923880f, -0.914210f, -0.903989f, -0.893224f, -0.881921f, -0.870087f, -0.857729f, -0.844854f,
    -0.831470f, -0.817585f, -0.803208f, -0.788346f, -0.773010f, -0.757209f, -0.740951f, -0.724247f,
    -0.707107f, -0.689541f, -0.671559f, -0.653173f, -0.634393f, -0.615232f, -0.595699f, -0.575808f,
    -0.555570f, -0.534998f, -0.514103f, -0.492898f, -0.471397f, -0.449611f, -0.427555f, -0.405241f,
    -0.382683f, -0.359895f, -0.336890f, -0.313682f, -0.290285f, -0.266713f, -0.242980f, -0.219101f,
    -0.195090f, -0.170962f, -0.146730f, -0.122411f, -0.098017f, -0.073565f, -0.049068f, -0.024541f
};

/* Private helper: clamp float value to [min, max] */
static inline float fclamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* Private helper: wrap angle to [0, 2π) */
static inline float fwrap_2pi(float theta)
{
    while (theta >= FOC_2PI) theta -= FOC_2PI;
    while (theta < 0.0f)     theta += FOC_2PI;
    return theta;
}

/* Private helper: get sin/cos from lookup table by float angle (radians) */
static inline void f32_sincos_lut(float theta, float *sin_val, float *cos_val)
{
    /* Map theta [0, 2π) → table index [0, 256) */
    uint8_t idx = ((uint32_t)(theta * FOC_ANGLE_TO_IDX_SCALE)) & 0xFFU;
    *sin_val = FOC_SinTable_F32[idx];
    *cos_val = FOC_SinTable_F32[((uint16_t)idx + 64U) & 0xFFU];
}

/* Exported variables --------------------------------------------------------*/

/** @brief FOC 全局电机状态 */
FOC_MotorState g_stMotor;

/** @brief 电流零点 ADC 偏移（浮点，ADC 原始计数值） */
float FOC_fIaOffsetAdc = FOC_IA_OFFSET_ADC;
float FOC_fIbOffsetAdc = FOC_IB_OFFSET_ADC;

/** @brief PI 控制器实例 */
FOC_PI g_stPiId;
FOC_PI g_stPiIq;
FOC_PI g_stPiSpeed;

/** @brief 龙伯格观测器实例 */
FOC_Luenberger g_stLuenberger;

/** @brief FOC 控制状态机 */
FOC_ControlState g_stCtrl;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  SVPWM 计算 — 共模注入法 + xyz 扇区判断
  * @param  fValpha  α轴电压分量 (pu, [-1.0, 1.0])
  * @param  fVbeta   β轴电压分量 (pu, [-1.0, 1.0])
  * @param  pu16Ta     A相PWM比较值输出
  * @param  pu16Tb     B相PWM比较值输出
  * @param  pu16Tc     C相PWM比较值输出
  * @param  pu16Sector 扇区号输出 (1~6)
  */
void FOC_Svpwm(float fValpha, float fVbeta,
               uint16_t *pu16Ta, uint16_t *pu16Tb,
               uint16_t *pu16Tc, uint16_t *pu16Sector)
{
    float fVa, fVb, fVc;
    float fVmin, fVmax, fVoffset;
    float fArr = (float)FOC_PWM_ARR;   /* 8500, center-aligned ARR */
    float fTa, fTb, fTc;
    float fX, fY, fZ;   /* xyz for sector detection */

    /*--- 步骤1: Clarke反变换 → 三相参考电压 ---*/
    fVa = fValpha;
    fVb = -0.5f * fValpha + 0.8660254037844386f * fVbeta;  /* -1/2·Vα + √3/2·Vβ */
    fVc = -0.5f * fValpha - 0.8660254037844386f * fVbeta;

    /*--- 步骤2: 共模注入 (SVPWM 等效) ---*/
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

    /*--- 步骤3: 缩放至 PWM 比较值 [0, ARR] ---*/
    fTa = (fVa + 1.0f) * 0.5f * fArr;
    fTb = (fVb + 1.0f) * 0.5f * fArr;
    fTc = (fVc + 1.0f) * 0.5f * fArr;

    /* 限幅并输出 */
    *pu16Ta = (uint16_t)fclamp(fTa, 0.0f, fArr);
    *pu16Tb = (uint16_t)fclamp(fTb, 0.0f, fArr);
    *pu16Tc = (uint16_t)fclamp(fTc, 0.0f, fArr);

    /*--- 步骤4: xyz 法判断扇区 ---*/
    fX = fVbeta;
    fY = 0.5f * (FOC_SQRT3 * fValpha + fVbeta);
    fZ = 0.5f * (-FOC_SQRT3 * fValpha + fVbeta);

    if (fY < 0.0f)
    {
        if (fZ < 0.0f)           *pu16Sector = 5;
        else if (fX < 0.0f)      *pu16Sector = 4;
        else                     *pu16Sector = 3;
    }
    else
    {
        if (fZ > 0.0f)           *pu16Sector = 2;
        else if (fX < 0.0f)      *pu16Sector = 6;
        else                     *pu16Sector = 1;
    }
}

/**
  * @brief  Clarke 变换（浮点版）
  *         Iα = Ia
  *         Iβ = (Ia + 2×Ib) / √3
  */
void FOC_Clarke(float fIa, float fIb,
                float *pfIalpha, float *pfIbeta)
{
    *pfIalpha = fIa;
    *pfIbeta  = (fIa + 2.0f * fIb) * FOC_INV_SQRT3;
}

/**
  * @brief  Park 变换（浮点版）
  *         Id =  Iα·cosθ + Iβ·sinθ
  *         Iq = -Iα·sinθ + Iβ·cosθ
  */
void FOC_Park(float fIalpha, float fIbeta, float fTheta,
              float *pfId, float *pfIq)
{
    float fSin, fCos;
    f32_sincos_lut(fTheta, &fSin, &fCos);
    *pfId =  fIalpha * fCos + fIbeta * fSin;
    *pfIq = -fIalpha * fSin + fIbeta * fCos;
}

/**
  * @brief  反 Park 变换（浮点版）
  *         Vα = Vd·cosθ - Vq·sinθ
  *         Vβ = Vd·sinθ + Vq·cosθ
  */
void FOC_InvPark(float fVd, float fVq, float fTheta,
                 float *pfValpha, float *pfVbeta)
{
    float fSin, fCos;
    f32_sincos_lut(fTheta, &fSin, &fCos);
    *pfValpha = fVd * fCos - fVq * fSin;
    *pfVbeta  = fVd * fSin + fVq * fCos;
}

/**
  * @brief  VF 控制器初始化
  */
void FOC_VF_Init(FOC_VF_State *pstVf)
{
    pstVf->f32Theta = 0.0f;
}

/**
  * @brief  VF 控制器单步运行（浮点版）
  * @param  pstVf       VF 控制器状态指针
  * @param  fTargetRpm  目标机械转速 (RPM)
  * @param  pfUd        d轴电压输出指针 (pu)
  * @param  pfUq        q轴电压输出指针 (pu)
  */
void FOC_VF_Run(FOC_VF_State *pstVf, float fTargetRpm,
                float *pfUd, float *pfUq)
{
    float fStep;

    *pfUd = 0.0f;

    /* Uq = VF_BOOST + VF_SLOPE × targetRpm */
    *pfUq = FOC_VF_BOOST + FOC_VF_SLOPE * fTargetRpm;
    if (*pfUq > FOC_VOLTAGE_MAX_PU)  *pfUq = FOC_VOLTAGE_MAX_PU;
    if (*pfUq < -FOC_VOLTAGE_MAX_PU) *pfUq = -FOC_VOLTAGE_MAX_PU;

    /* 角度积分: θ_step = RPM × 2π × pole_pairs / 60 / FOC_CTRL_FREQ */
    fStep = fTargetRpm * (FOC_2PI * (float)MOTOR_POLE_PAIRS / 60.0f) * FOC_CTRL_TS;
    pstVf->f32Theta = fwrap_2pi(pstVf->f32Theta + fStep);
}

/*----------------------------------------------------------------------------*/
/* FOC 初始化                                                                 */
/*----------------------------------------------------------------------------*/
void FOC_Init(void)
{
    memset(&g_stMotor, 0, sizeof(g_stMotor));

    FOC_StateMachine_Init();

    /* Id 电流环 PI */
    g_stPiId.fKp        = FOC_PI_ID_KP;
    g_stPiId.fKi        = FOC_PI_ID_KI;
    g_stPiId.fIntegral  = 0.0f;
    g_stPiId.fOutMax    =  FOC_VOLTAGE_MAX_PU;
    g_stPiId.fOutMin    = -FOC_VOLTAGE_MAX_PU;

    /* Iq 电流环 PI */
    g_stPiIq.fKp        = FOC_PI_IQ_KP;
    g_stPiIq.fKi        = FOC_PI_IQ_KI;
    g_stPiIq.fIntegral  = 0.0f;
    g_stPiIq.fOutMax    =  FOC_VOLTAGE_MAX_PU;
    g_stPiIq.fOutMin    = -FOC_VOLTAGE_MAX_PU;

    /* 速度环 PI */
    g_stPiSpeed.fKp       = FOC_PI_SPEED_KP;
    g_stPiSpeed.fKi       = FOC_PI_SPEED_KI;
    g_stPiSpeed.fIntegral = 0.0f;
    g_stPiSpeed.fOutMax   =  FOC_PI_SPEED_OUT_MAX;
    g_stPiSpeed.fOutMin   =  FOC_PI_SPEED_OUT_MIN;

    FOC_Luenberger_Init();
}

/*----------------------------------------------------------------------------*/
/* PI 控制器（浮点版）                                                        */
/*----------------------------------------------------------------------------*/
float FOC_PI_Run(FOC_PI *pstPi, float fRef, float fFb)
{
    float fErr = fRef - fFb;
    float fOut;

    /* 积分累加（Ki 已含采样时间） */
    pstPi->fIntegral += pstPi->fKi * fErr;

    /* 抗饱和限幅 */
    if (pstPi->fIntegral > pstPi->fOutMax) pstPi->fIntegral = pstPi->fOutMax;
    if (pstPi->fIntegral < pstPi->fOutMin) pstPi->fIntegral = pstPi->fOutMin;

    /* 比例 + 积分 */
    pstPi->fProportional = pstPi->fKp * fErr;
    fOut = pstPi->fProportional + pstPi->fIntegral;

    /* 输出限幅 */
    if (fOut > pstPi->fOutMax) fOut = pstPi->fOutMax;
    if (fOut < pstPi->fOutMin) fOut = pstPi->fOutMin;

    return fOut;
}

/*----------------------------------------------------------------------------*/
/* 状态机初始化                                                               */
/*----------------------------------------------------------------------------*/
void FOC_StateMachine_Init(void)
{
    memset(&g_stCtrl, 0, sizeof(g_stCtrl));

    g_stCtrl.eMode         = FOC_MODE_IDLE;
    g_stCtrl.eModePrev     = FOC_MODE_IDLE;
    g_stCtrl.f32TargetRpm  = 0.0f;
    g_stCtrl.f32IdRef      = 0.0f;
    g_stCtrl.f32IqRef      = FOC_CTRL_IQ_STARTUP;
    g_stCtrl.f32RpmRamp    = 0.0f;
}

/*----------------------------------------------------------------------------*/
/* 状态机运行（Imperix 五阶段切换法，浮点版）                                 */
/*----------------------------------------------------------------------------*/
void FOC_StateMachine_Run(void)
{
    float fStep;

    switch (g_stCtrl.eMode)
    {
        case FOC_MODE_IDLE:
            if (g_stCtrl.f32TargetRpm > 0.0f)
            {
                g_stCtrl.f32RpmRamp       = 0.0f;
                g_stCtrl.u16AlignCount    = 0;
                g_stCtrl.u16AlignFrames   = 0;
                g_stCtrl.f32IqRampDown    = 0.0f;
                g_stCtrl.u16SwitchStableCount = 0;
                g_stCtrl.u16BlendCount    = 0;
                g_stCtrl.u16SpeedStableCount  = 0;
                g_stCtrl.u16ObsLostCount  = 0;

                g_stPiId.fIntegral    = 0.0f;
                g_stPiIq.fIntegral    = 0.0f;
                g_stPiSpeed.fIntegral = 0.0f;

                g_stMotor.f32Theta = 0.0f;

                g_stCtrl.eModePrev = g_stCtrl.eMode;
                g_stCtrl.eMode = FOC_MODE_IF_STARTUP;
            }
            break;

        case FOC_MODE_IF_STARTUP:
        {
            /*--- 阶段2: IF 开环启动 ---*/
            g_stCtrl.u16AlignFrames++;

            /* Iq 软启动：从 ALIGN_START 斜坡到 STARTUP */
            {
                float fRampFrames = (float)FOC_CTRL_ALIGN_DURATION;
                if ((float)g_stCtrl.u16AlignFrames < fRampFrames)
                {
                    float fProgress = (float)g_stCtrl.u16AlignFrames / fRampFrames;
                    g_stCtrl.f32IqRef = FOC_CTRL_IQ_ALIGN_START
                                      + (FOC_CTRL_IQ_STARTUP - FOC_CTRL_IQ_ALIGN_START) * fProgress;
                }
                else
                {
                    g_stCtrl.f32IqRef = FOC_CTRL_IQ_STARTUP;
                }
            }

            /* θ 斜坡加速 */
            g_stCtrl.u16AlignCount++;
#ifdef FOC_IF_OPENLOOP_DEBUG
            /*--- IF 开环调试：直接斜坡到目标转速，不切换 ---*/
            if ((g_stCtrl.f32RpmRamp < g_stCtrl.f32TargetRpm) &&
                (g_stCtrl.u16AlignCount >= FOC_CTRL_RPM_ACCEL_INTERVAL))
            {
                g_stCtrl.f32RpmRamp += FOC_CTRL_RPM_ACCEL_STEP;
                if (g_stCtrl.f32RpmRamp > g_stCtrl.f32TargetRpm)
                    g_stCtrl.f32RpmRamp = g_stCtrl.f32TargetRpm;
                g_stCtrl.u16AlignCount = 0;
            }
#else
            if ((g_stCtrl.f32RpmRamp < FOC_CTRL_RPM_SWITCH) &&
                (g_stCtrl.u16AlignCount >= FOC_CTRL_RPM_ACCEL_INTERVAL))
            {
                g_stCtrl.f32RpmRamp += FOC_CTRL_RPM_ACCEL_STEP;
                g_stCtrl.u16AlignCount = 0;
            }
#endif

            fStep = g_stCtrl.f32RpmRamp * (FOC_2PI * (float)MOTOR_POLE_PAIRS / 60.0f) * FOC_CTRL_TS;
            g_stMotor.f32Theta = fwrap_2pi(g_stMotor.f32Theta + fStep);

            g_stCtrl.f32IdRef = 0.0f;

#ifndef FOC_IF_OPENLOOP_DEBUG
            if (g_stCtrl.f32RpmRamp >= FOC_CTRL_RPM_SWITCH)
            {
                g_stCtrl.eModePrev = g_stCtrl.eMode;
                g_stCtrl.eMode = FOC_MODE_IF_ALIGN;
                g_stCtrl.u16AlignCount = 0;
                g_stCtrl.f32IqRampDown = FOC_CTRL_IQ_STARTUP;
            }
#endif
            break;
        }

        case FOC_MODE_IF_ALIGN:
        {
            /*--- 阶段3: 恒速等待 ---*/
            fStep = FOC_CTRL_RPM_SWITCH * (FOC_2PI * (float)MOTOR_POLE_PAIRS / 60.0f) * FOC_CTRL_TS;
            g_stMotor.f32Theta = fwrap_2pi(g_stMotor.f32Theta + fStep);

            g_stCtrl.u16AlignFrames++;
            g_stCtrl.f32IqRef = FOC_CTRL_IQ_STARTUP;
            g_stCtrl.f32IdRef = 0.0f;

            if (g_stCtrl.u16AlignFrames >= FOC_CTRL_RPM_SETTLE_FRAMES)
            {
                float fSpdObs = g_stLuenberger.f32SpeedObs;
                float fSwSpd  = FOC_CTRL_RPM_SWITCH;
                float fTol    = fSwSpd * (float)FOC_CTRL_SPEED_TOL_PCT / 100.0f;
                uint8_t u8SpeedOk = (fSpdObs > (fSwSpd - fTol) && fSpdObs < (fSwSpd + fTol)) ? 1 : 0;

                if (u8SpeedOk)
                {
                    g_stCtrl.u16SpeedStableCount++;
                    if (g_stCtrl.u16SpeedStableCount >= FOC_CTRL_STABLE_COUNT_THR)
                    {
                        /* 保存角度差（弧度，带符号） */
                        float fErr = g_stLuenberger.f32ThetaObs - g_stMotor.f32Theta;
                        /* 将角度差归一到 [-π, π) */
                        while (fErr > FOC_MATH_PI)  fErr -= FOC_2PI;
                        while (fErr < -FOC_MATH_PI) fErr += FOC_2PI;
                        g_stCtrl.f32ThetaErrSave = fErr;

                        g_stCtrl.eModePrev = g_stCtrl.eMode;
                        g_stCtrl.eMode = FOC_MODE_TRANSITION;
                        g_stCtrl.u16BlendCount = 0;
                        g_stCtrl.f32ThetaIfRef = g_stMotor.f32Theta;
                        g_stCtrl.u16DiagTransitionFlag = 1;
                    }
                }
                else
                {
                    g_stCtrl.u16SpeedStableCount = 0;
                }
            }

            /* 更新转矩角 (rad) */
            {
                float fTq = g_stLuenberger.f32ThetaObs - g_stMotor.f32Theta;
                while (fTq < 0.0f) fTq += FOC_2PI;
                while (fTq >= FOC_2PI) fTq -= FOC_2PI;
                g_stCtrl.f32TorqueAngle = fTq;
            }
            break;
        }

        case FOC_MODE_TRANSITION:
        {
            /*--- 阶段4: 误差递减 ---*/
            g_stCtrl.u16BlendCount++;

            {
                float fCnt = (float)g_stCtrl.u16BlendCount;
                float fTotal = (float)FOC_CTRL_TRANSITION_FRAMES;
                if (fCnt > fTotal) fCnt = fTotal;

                float fRemain = g_stCtrl.f32ThetaErrSave * (fTotal - fCnt) / fTotal;
                g_stMotor.f32Theta = g_stLuenberger.f32ThetaObs - fRemain;
                g_stMotor.f32Theta = fwrap_2pi(g_stMotor.f32Theta);
            }

            if (g_stCtrl.u16BlendCount >= FOC_CTRL_TRANSITION_FRAMES)
            {
                g_stMotor.f32Theta = g_stLuenberger.f32ThetaObs;

                /* 预载速度 PI 积分：速度 PI 内部输出为 Q12 计数，进入电流环前转 pu。 */
                float fSpdErr  = g_stCtrl.f32TargetRpm - g_stLuenberger.f32SpeedObs;
                float fKpTerm  = FOC_PI_SPEED_KP * fSpdErr;

                float fCurSpd = g_stLuenberger.f32SpeedObs;
                if (fCurSpd < 1.0f) fCurSpd = 1.0f;
                float fIqEst = g_stCtrl.f32IqRef * FOC_CTRL_RPM_SWITCH / fCurSpd;
                if (fIqEst > g_stCtrl.f32IqRef) fIqEst = g_stCtrl.f32IqRef;
                if (fIqEst < FOC_Q12_TO_PU(1000.0f))
                    fIqEst = FOC_Q12_TO_PU(1000.0f);

                g_stPiSpeed.fIntegral = FOC_PU_TO_Q12(fIqEst) - fKpTerm;
                if (g_stPiSpeed.fIntegral > g_stPiSpeed.fOutMax)
                    g_stPiSpeed.fIntegral = g_stPiSpeed.fOutMax;
                if (g_stPiSpeed.fIntegral < g_stPiSpeed.fOutMin)
                    g_stPiSpeed.fIntegral = g_stPiSpeed.fOutMin;

                g_stCtrl.eModePrev = g_stCtrl.eMode;
                g_stCtrl.eMode = FOC_MODE_CLOSED_LOOP;
                g_stCtrl.u16DiagTransitionFlag = 0;
                g_stCtrl.u16SwitchStableCount = 0;
                g_stCtrl.u16ObsLostCount = 0;
            }
            break;
        }

        case FOC_MODE_CLOSED_LOOP:
            /*--- 阶段5: 闭环运行 ---*/
            g_stMotor.f32Theta = g_stLuenberger.f32ThetaObs;

            if (g_stCtrl.u16SwitchStableCount < FOC_CTRL_STABLE_FRAMES)
            {
                g_stCtrl.u16SwitchStableCount++;
            }
            else if (!g_stLuenberger.u16Locked)
            {
                g_stCtrl.u16ObsLostCount++;
                if (g_stCtrl.u16ObsLostCount >= FOC_CTRL_OBS_LOST_THRESHOLD)
                {
                    g_stCtrl.eModePrev = g_stCtrl.eMode;
                    g_stCtrl.eMode = FOC_MODE_IDLE;
                    g_stCtrl.u16ObsLostCount = 0;
                    g_stCtrl.f32TargetRpm = 0.0f;
                    g_stPiId.fIntegral    = 0.0f;
                    g_stPiIq.fIntegral    = 0.0f;
                    g_stPiSpeed.fIntegral = 0.0f;
                }
            }
            else
            {
                g_stCtrl.u16ObsLostCount = 0;
            }
            break;

        case FOC_MODE_FAULT:
            break;
    }

    /*--- 停止指令：回到 IDLE ---*/
    if (g_stCtrl.f32TargetRpm == 0.0f)
    {
        if (g_stCtrl.eMode != FOC_MODE_IDLE)
        {
            g_stCtrl.eModePrev = g_stCtrl.eMode;
            g_stCtrl.eMode = FOC_MODE_IDLE;
        }
    }
}

/*----------------------------------------------------------------------------*/
/* FOC 控制步进（10kHz 中断调用，浮点版）                                     */
/*----------------------------------------------------------------------------*/
void FOC_ControlStep(void)
{
    static uint16_t s_u16SpeedLoopCnt = 0;

    /*--- 第0步：状态机运行 ---*/
    FOC_StateMachine_Run();

    if ((g_stCtrl.eMode == FOC_MODE_IDLE) || (g_stCtrl.eMode == FOC_MODE_FAULT))
    {
        s_u16SpeedLoopCnt = 0;
        TIM1->CCR1 = PWM_HALF_CYCLE;
        TIM1->CCR2 = PWM_HALF_CYCLE;
        TIM1->CCR3 = PWM_HALF_CYCLE;
        return;
    }

    /*--- 速度 PI（分频执行 10kHz/10=1kHz） ---*/
    if (g_stCtrl.eMode == FOC_MODE_CLOSED_LOOP)
    {
        s_u16SpeedLoopCnt++;
        if (s_u16SpeedLoopCnt >= FOC_CTRL_SPEED_LOOP_DECIMATION)
        {
            s_u16SpeedLoopCnt = 0;

            float fSpdErr = g_stCtrl.f32TargetRpm - g_stLuenberger.f32SpeedObs;

            /* 积分分离：|误差| > 100 RPM 时临时 Ki=0 */
            if (fSpdErr > 100.0f || fSpdErr < -100.0f)
            {
                g_stPiSpeed.fKi = 0.0f;
            }
            else
            {
                g_stPiSpeed.fKi = FOC_PI_SPEED_KI;
            }

            g_stCtrl.f32IqRef = FOC_Q12_TO_PU(FOC_PI_Run(&g_stPiSpeed,
                g_stCtrl.f32TargetRpm, g_stLuenberger.f32SpeedObs));

            /* 保存速度 PI 比例项用于诊断 */
            g_stCtrl.f32ThetaErrSave = FOC_Q12_TO_PU(FOC_PI_SPEED_KP * fSpdErr);
        }
    }
    else
    {
        s_u16SpeedLoopCnt = 0;
    }

    /*--- 第1步：Clarke 变换 ---*/
    FOC_Clarke(g_stMotor.f32Ia, g_stMotor.f32Ib,
               &g_stMotor.f32Ialpha, &g_stMotor.f32Ibeta);

    /*--- 第2步：Park 变换 ---*/
    FOC_Park(g_stMotor.f32Ialpha, g_stMotor.f32Ibeta,
             g_stMotor.f32Theta,
             &g_stMotor.f32Id, &g_stMotor.f32Iq);

    /*--- 第3步：PI 电流闭环 ---*/
    g_stMotor.f32UdRef = FOC_PI_Run(&g_stPiId, g_stCtrl.f32IdRef, g_stMotor.f32Id);
    g_stMotor.f32UqRef = FOC_PI_Run(&g_stPiIq, g_stCtrl.f32IqRef, g_stMotor.f32Iq);

    /*--- 第4步：反 Park 变换 ---*/
    FOC_InvPark(g_stMotor.f32UdRef, g_stMotor.f32UqRef,
                g_stMotor.f32Theta,
                &g_stMotor.f32Valpha, &g_stMotor.f32Vbeta);

    /*--- 第5步：SVPWM ---*/
    FOC_Svpwm(g_stMotor.f32Valpha, g_stMotor.f32Vbeta,
              &g_stMotor.u16Ta, &g_stMotor.u16Tb,
              &g_stMotor.u16Tc, &g_stMotor.u16Sector);

    /*--- 第6步：更新 TIM1 CCR ---*/
    TIM1->CCR1 = g_stMotor.u16Ta;
    TIM1->CCR2 = g_stMotor.u16Tb;
    TIM1->CCR3 = g_stMotor.u16Tc;

    /*--- 第7步：龙伯格观测器 ---*/
#ifndef FOC_IF_OPENLOOP_DEBUG
    FOC_Luenberger_Run();

    /*--- 更新转矩角 (rad) ---*/
    {
        float fTmp = g_stLuenberger.f32ThetaObs - g_stMotor.f32Theta;
        while (fTmp < 0.0f)       fTmp += FOC_2PI;
        while (fTmp >= FOC_2PI)   fTmp -= FOC_2PI;
        g_stCtrl.f32TorqueAngle = fTmp;
    }
#else
    /* IF 开环调试模式：观测器不运行，转矩角置 90° 占位 */
    g_stCtrl.f32TorqueAngle = FOC_PI_2;
#endif
}

/*----------------------------------------------------------------------------*/
/* ADC 电流采样 → 浮点 pu 电流                                               */
/*----------------------------------------------------------------------------*/
void FOC_GetPhaseCurrent(void)
{
    uint16_t u16IaRaw;
    uint16_t u16IbRaw;
    float fIa, fIb;

    /* 禁止 CH4 触发，防止重入 */
    TIM1->CCER &= ~TIM_CCER_CC4E;

    /* 读取 ADC 注入数据寄存器（12位右对齐，0~4095） */
    u16IaRaw = (uint16_t)(ADC1->JDR1);
    u16IbRaw = (uint16_t)(ADC2->JDR1);

    /* 转换：current_pu = (offset - raw) / scale
     * offset = ADC零电流读数, scale = ADC计数值对应 1.0pu
     * 方向修正：采样电阻在低端，用 offset-raw 实现反转 */
    fIa = (FOC_fIaOffsetAdc - (float)u16IaRaw) / FOC_ADC_CURRENT_SCALE;
    fIb = (FOC_fIbOffsetAdc - (float)u16IbRaw) / FOC_ADC_CURRENT_SCALE;

    /* 限幅到 ±1.0 pu */
    g_stMotor.f32Ia = fclamp(fIa, -1.0f, 1.0f);
    g_stMotor.f32Ib = fclamp(fIb, -1.0f, 1.0f);

    /* Ic = -(Ia + Ib) */
    g_stMotor.f32Ic = fclamp(-(g_stMotor.f32Ia + g_stMotor.f32Ib), -1.0f, 1.0f);
}

/*----------------------------------------------------------------------------*/
/* 龙伯格观测器初始化                                                         */
/*----------------------------------------------------------------------------*/
void FOC_Luenberger_Init(void)
{
    FOC_Luenberger *pstObs = &g_stLuenberger;

    pstObs->f32IalphaEst = 0.0f;
    pstObs->f32IbetaEst  = 0.0f;
    pstObs->f32Ealpha    = 0.0f;
    pstObs->f32Ebeta     = 0.0f;
    pstObs->f32SpeedObs  = 0.0f;
    pstObs->f32ThetaObs  = 0.0f;
    pstObs->u16Locked    = 0;
}

/*----------------------------------------------------------------------------*/
/* 龙伯格观测器运行（浮点版）                                                 */
/*----------------------------------------------------------------------------*/
void FOC_Luenberger_Run(void)
{
    FOC_Luenberger *pstObs = &g_stLuenberger;
    static float s_fIalphaPrev = 0.0f;
    static float s_fIbetaPrev  = 0.0f;
    float fEalpha, fEbeta;
    float fDiAlpha, fDiBeta;
    float fMagSq;

    /*--- 第1步：电流微分 ---*/
    /*   di_pu = I_pu[n] - I_pu[n-1] (ΔI per step)
     *   电感补偿项 = (Ls_pu / Ts_pu) × di_pu
     *   其中 Ls_pu/Ts_pu ≈ 7.572
     */
    #define FOC_LS_OVER_TS_PU    (FOC_LS_PU / FOC_TS_PU)  /* ≈ 7.572 */

    fDiAlpha = g_stMotor.f32Ialpha - s_fIalphaPrev;
    fDiBeta  = g_stMotor.f32Ibeta  - s_fIbetaPrev;

    /*--- 第2步：反电势估计 ---*/
    /* e = V - Rs·I - (Ls/Ts)·ΔI  (全部 pu) */
    fEalpha = g_stMotor.f32Valpha
            - FOC_RS_PU * g_stMotor.f32Ialpha
            - FOC_LS_OVER_TS_PU * fDiAlpha;

    fEbeta  = g_stMotor.f32Vbeta
            - FOC_RS_PU * g_stMotor.f32Ibeta
            - FOC_LS_OVER_TS_PU * fDiBeta;

    s_fIalphaPrev = g_stMotor.f32Ialpha;
    s_fIbetaPrev  = g_stMotor.f32Ibeta;

    /*--- 第3步：低通滤波（一阶 IIR, α=G1=0.03125） ---*/
    fEalpha = pstObs->f32Ealpha + FOC_OBS_G1 * (fEalpha - pstObs->f32Ealpha);
    fEbeta  = pstObs->f32Ebeta  + FOC_OBS_G1 * (fEbeta  - pstObs->f32Ebeta);

    /* 限幅 */
    fEalpha = fclamp(fEalpha, -0.98f, 0.98f);
    fEbeta  = fclamp(fEbeta,  -0.98f, 0.98f);

    pstObs->f32Ealpha = fEalpha;
    pstObs->f32Ebeta  = fEbeta;

    /*--- 第4步：PLL 角度跟踪 ---*/
    {
        static float s_fOmegaPll = 0.0f;  /* 电气角速度 (rad/s) */
        float fSin, fCos;
        float fErr;

        /* 查表获取 cos(θ̂), sin(θ̂) */
        f32_sincos_lut(pstObs->f32ThetaObs, &fSin, &fCos);

        /* PLL 误差：BEMF 在 q 轴上的投影
         *   e_q = -eα·cos(θ̂) - eβ·sin(θ̂)
         *   稳态时 e_q=0，有误差时 e_q ∝ sin(Δθ) */
        fErr = -fEalpha * fCos - fEbeta * fSin;
        fErr = fclamp(fErr, -FOC_PLL_ERR_MAX, FOC_PLL_ERR_MAX);

        /* 积分：ω += Ki × err */
        s_fOmegaPll += FOC_PLL_KI * fErr;
        if (s_fOmegaPll < 0.0f) s_fOmegaPll = 0.0f;
        if (s_fOmegaPll > FOC_PLL_OMEGA_MAX) s_fOmegaPll = FOC_PLL_OMEGA_MAX;

        /* 角度更新：θ += ω×Ts + Kp×err */
        {
            float fStep = s_fOmegaPll * FOC_CTRL_TS + FOC_PLL_KP * fErr;
            pstObs->f32ThetaObs = fwrap_2pi(pstObs->f32ThetaObs + fStep);
        }

        /* 转速估计 (RPM) */
        pstObs->f32SpeedObs = s_fOmegaPll * FOC_ELEC_OMEGA_TO_RPM;
        if (pstObs->f32SpeedObs > 10000.0f) pstObs->f32SpeedObs = 10000.0f;

        /* 诊断 */
        pstObs->f32ErrPll = fErr;
    }

    /*--- 保存 BEMF 幅值诊断 ---*/
    {
        float a = pstObs->f32Ealpha;
        float b = pstObs->f32Ebeta;
        if (a < 0.0f) a = -a;
        if (b < 0.0f) b = -b;
        /* 快速近似 sqrt: max + 0.5*min */
        if (a < b) { float t = a; a = b; b = t; }
        pstObs->f32BemfMag = a + 0.5f * b;
    }

    /*--- 第5步：锁定判断 ---*/
    fMagSq = pstObs->f32Ealpha * pstObs->f32Ealpha
           + pstObs->f32Ebeta  * pstObs->f32Ebeta;

    pstObs->u16Locked = (fMagSq > EMF_LOCK_SQ_THR_F
                         && pstObs->f32SpeedObs > EMF_LOCK_SPD_THR_F) ? 1 : 0;
}
