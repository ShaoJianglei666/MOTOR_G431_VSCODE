/**
  ******************************************************************************
  * @file    FOC.c
  * @brief   FOC IF 开环控制 — 精简版 【v3.0】
  *          纯 IF 开环：角度开环积分 + 电流闭环 PI + SVPWM
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FOC.h"
#include "main.h"
#include <string.h>

/*============================================================================*/
/* 正弦查找表 (256点, float)                                                    */
/*============================================================================*/
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

/*============================================================================*/
/* 私有辅助函数                                                                */
/*============================================================================*/

static inline float fclamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float fwrap_2pi(float theta)
{
    while (theta >= FOC_2PI) theta -= FOC_2PI;
    while (theta < 0.0f)     theta += FOC_2PI;
    return theta;
}

static inline void flimit_vector(float *x, float *y, float limit)
{
    float mag_sq = (*x * *x) + (*y * *y);
    float limit_sq = limit * limit;

    if (mag_sq > limit_sq)
    {
        float scale = limit / sqrtf(mag_sq);
        *x *= scale;
        *y *= scale;
    }
}

static inline float FOC_GetBemfFeedForwardPu(void)
{
    return g_stCtrl.f32RpmRamp * FOC_BEMF_PU_PER_RPM;
}

static inline float FOC_GetDynamicVoltageLimitPu(void)
{
    float limit = FOC_VOLTAGE_MIN_PU
                + FOC_GetBemfFeedForwardPu()
                + FOC_VOLTAGE_MARGIN_PU;

    return fclamp(limit, FOC_VOLTAGE_MIN_PU, FOC_VOLTAGE_MAX_PU);
}

static inline void f32_sincos_lut(float theta, float *sin_val, float *cos_val)
{
    uint8_t idx = ((uint32_t)(theta * FOC_ANGLE_TO_IDX_SCALE)) & 0xFFU;
    *sin_val = FOC_SinTable_F32[idx];
    *cos_val = FOC_SinTable_F32[((uint16_t)idx + 64U) & 0xFFU];
}

/*============================================================================*/
/* 全局变量                                                                    */
/*============================================================================*/

FOC_MotorState   g_stMotor;
FOC_ControlState g_stCtrl;
FOC_PI           g_stPiId;
FOC_PI           g_stPiIq;
float            FOC_fIaOffsetAdc = 1972.0f;
float            FOC_fIbOffsetAdc = 1972.0f;

/*============================================================================*/
/* SVPWM — 共模注入法                                                          */
/*============================================================================*/
void FOC_Svpwm(float fValpha, float fVbeta,
               uint16_t *pu16Ta, uint16_t *pu16Tb, uint16_t *pu16Tc)
{
    float fVa, fVb, fVc;
    float fVmin, fVmax, fVoffset;
    float fArr = (float)FOC_PWM_ARR;

    /* Clarke反变换 */
    fVa = fValpha;
    fVb = -0.5f * fValpha + 0.8660254037844386f * fVbeta;
    fVc = -0.5f * fValpha - 0.8660254037844386f * fVbeta;

    /* 共模注入 */
    fVmin = fVa; if (fVb < fVmin) fVmin = fVb; if (fVc < fVmin) fVmin = fVc;
    fVmax = fVa; if (fVb > fVmax) fVmax = fVb; if (fVc > fVmax) fVmax = fVc;
    fVoffset = -0.5f * (fVmin + fVmax);

    fVa += fVoffset; fVb += fVoffset; fVc += fVoffset;

    /* 缩放至 [0, ARR] */
    *pu16Ta = (uint16_t)fclamp((fVa + 1.0f) * 0.5f * fArr, 0.0f, fArr);
    *pu16Tb = (uint16_t)fclamp((fVb + 1.0f) * 0.5f * fArr, 0.0f, fArr);
    *pu16Tc = (uint16_t)fclamp((fVc + 1.0f) * 0.5f * fArr, 0.0f, fArr);
}

/*============================================================================*/
/* Clarke 变换: Ialpha=Ia, Ibeta=(Ia+2*Ib)/sqrt(3)                             */
/*============================================================================*/
void FOC_Clarke(float fIa, float fIb, float *pfIalpha, float *pfIbeta)
{
    *pfIalpha = fIa;
    *pfIbeta  = (fIa + 2.0f * fIb) * FOC_INV_SQRT3;
}

/*============================================================================*/
/* Park 变换: Id=Ialpha*cos+Ibeta*sin, Iq=-Ialpha*sin+Ibeta*cos               */
/*============================================================================*/
void FOC_Park(float fIalpha, float fIbeta, float fTheta,
              float *pfId, float *pfIq)
{
    float fSin, fCos;
    f32_sincos_lut(fTheta, &fSin, &fCos);
    *pfId =  fIalpha * fCos + fIbeta * fSin;
    *pfIq = -fIalpha * fSin + fIbeta * fCos;
}

/*============================================================================*/
/* 反Park 变换: Valpha=Vd*cos-Vq*sin, Vbeta=Vd*sin+Vq*cos                     */
/*============================================================================*/
void FOC_InvPark(float fVd, float fVq, float fTheta,
                 float *pfValpha, float *pfVbeta)
{
    float fSin, fCos;
    f32_sincos_lut(fTheta, &fSin, &fCos);
    *pfValpha = fVd * fCos - fVq * fSin;
    *pfVbeta  = fVd * fSin + fVq * fCos;
}

/*============================================================================*/
/* PI 控制器                                                                    */
/*============================================================================*/
float FOC_PI_Run(FOC_PI *pstPi, float fRef, float fFb)
{
    float fErr = fRef - fFb;
    float fP;
    float fOut;
    float fIntegralNew;

    fP = pstPi->fKp * fErr;
    fOut = fP + pstPi->fIntegral;

    /* 饱和时只允许积分往退出饱和的方向走，避免错误反馈时快速打满。 */
    if (!((fOut >= pstPi->fOutMax && fErr > 0.0f) ||
          (fOut <= pstPi->fOutMin && fErr < 0.0f)))
    {
        fIntegralNew = pstPi->fIntegral + pstPi->fKi * fErr;
        pstPi->fIntegral = fclamp(fIntegralNew, pstPi->fOutMin, pstPi->fOutMax);
    }

    fOut = fP + pstPi->fIntegral;
    fOut = fclamp(fOut, pstPi->fOutMin, pstPi->fOutMax);

    return fOut;
}

/*============================================================================*/
/* FOC 初始化                                                                  */
/*============================================================================*/
void FOC_Init(void)
{
    memset(&g_stMotor, 0, sizeof(g_stMotor));
    memset(&g_stCtrl,  0, sizeof(g_stCtrl));

    g_stCtrl.eMode        = FOC_MODE_STOP;
    g_stCtrl.f32TargetRpm = 0.0f;

    /* Id PI */
    g_stPiId.fKp       = FOC_PI_ID_KP;
    g_stPiId.fKi       = FOC_PI_ID_KI;
    g_stPiId.fIntegral = 0.0f;
    g_stPiId.fOutMax   =  FOC_PI_CORRECTION_MAX_PU;
    g_stPiId.fOutMin   = -FOC_PI_CORRECTION_MAX_PU;

    /* Iq PI */
    g_stPiIq.fKp       = FOC_PI_IQ_KP;
    g_stPiIq.fKi       = FOC_PI_IQ_KI;
    g_stPiIq.fIntegral = 0.0f;
    g_stPiIq.fOutMax   =  FOC_PI_CORRECTION_MAX_PU;
    g_stPiIq.fOutMin   = -FOC_PI_CORRECTION_MAX_PU;
}

/*============================================================================*/
/* 状态机 — 极简版 (STOP / IF_OPENLOOP)                                        */
/*============================================================================*/
static void FOC_StateMachine_Run(void)
{
    float fStep;

    switch (g_stCtrl.eMode)
    {
    case FOC_MODE_STOP:
        /* 目标转速 > 0 → 进入 IF 开环 */
        if (g_stCtrl.f32TargetRpm > 0.0f)
        {
            g_stCtrl.f32RpmRamp   = 0.0f;
            g_stCtrl.u32RampCount = 0;
            g_stMotor.f32Theta    = 0.0f;

            g_stPiId.fIntegral = 0.0f;
            g_stPiIq.fIntegral = 0.0f;

            g_stCtrl.eMode = FOC_MODE_IF_OPENLOOP;
        }
        break;

    case FOC_MODE_IF_OPENLOOP:
    {
        /*--- 目标转速归零 → 回到 STOP ---*/
        if (g_stCtrl.f32TargetRpm == 0.0f)
        {
            g_stCtrl.eMode = FOC_MODE_STOP;
            break;
        }

        /*--- 阶段1：对齐 — Id 从 0 缓慢斜坡到 ALIGN_ID ---*/
        if (g_stCtrl.u32RampCount < FOC_IF_ALIGN_FRAMES)
        {
            float fProgress = (float)g_stCtrl.u32RampCount / (float)FOC_IF_ALIGN_FRAMES;
            g_stCtrl.f32IdRef = FOC_IF_ALIGN_ID * fProgress;  /* 0 -> ALIGN_ID */
            g_stCtrl.f32IqRef = 0.0f;
            /* 对齐阶段不旋转，θ 保持 0 */
            g_stMotor.f32Theta = 0.0f;
        }
        else
        {
            /*--- 阶段2：IF 开环运行 — Id 退出，Iq 进入 ---*/
            uint32_t u32RunFrames = g_stCtrl.u32RampCount - FOC_IF_ALIGN_FRAMES;
            if (u32RunFrames < FOC_IF_TRANSITION_FRAMES)
            {
                float fTrans = (float)u32RunFrames / (float)FOC_IF_TRANSITION_FRAMES;
                g_stCtrl.f32IdRef = FOC_IF_ALIGN_ID * (1.0f - fTrans);
                g_stCtrl.f32IqRef = FOC_IF_STARTUP_IQ * fTrans;
            }
            else
            {
                g_stCtrl.f32IdRef = 0.0f;
                g_stCtrl.f32IqRef = FOC_IF_STARTUP_IQ;
            }

            /*--- 转速斜坡 ---*/
            if ((g_stCtrl.f32RpmRamp < g_stCtrl.f32TargetRpm) &&
                ((u32RunFrames % FOC_IF_RAMP_INTERVAL) == 0) &&
                (u32RunFrames >= FOC_IF_TRANSITION_FRAMES))
            {
                g_stCtrl.f32RpmRamp += FOC_IF_RAMP_STEP_RPM;
                if (g_stCtrl.f32RpmRamp > g_stCtrl.f32TargetRpm)
                    g_stCtrl.f32RpmRamp = g_stCtrl.f32TargetRpm;
            }

            /*--- 角度积分 ---*/
            fStep = g_stCtrl.f32RpmRamp * (FOC_2PI * (float)MOTOR_POLE_PAIRS / 60.0f) * FOC_CTRL_TS;
            g_stMotor.f32Theta = fwrap_2pi(g_stMotor.f32Theta + fStep);
        }

        g_stCtrl.u32RampCount++;
        break;
    }
    }
}

/*============================================================================*/
/* FOC 控制步进 — 10kHz ADC 中断中调用                                         */
/*============================================================================*/
void FOC_ControlStep(void)
{
    float fUdPi;
    float fUqPi;
    float fUqFf;
    float fVoltLimit;

    /*--- 第0步：状态机 ---*/
    FOC_StateMachine_Run();

    /*--- STOP：输出 50% 占空比 ---*/
    if (g_stCtrl.eMode == FOC_MODE_STOP)
    {
        TIM1->CCR1 = PWM_HALF_CYCLE;
        TIM1->CCR2 = PWM_HALF_CYCLE;
        TIM1->CCR3 = PWM_HALF_CYCLE;
        return;
    }

    /*--- 第1步：Clarke ---*/
    FOC_Clarke(g_stMotor.f32Ia, g_stMotor.f32Ib,
               &g_stMotor.f32Ialpha, &g_stMotor.f32Ibeta);

    /*--- 第2步：Park ---*/
    FOC_Park(g_stMotor.f32Ialpha, g_stMotor.f32Ibeta,
             g_stMotor.f32Theta, &g_stMotor.f32Id, &g_stMotor.f32Iq);

    /*--- 第3步：速度电压前馈 + 小幅电流修正 ---
     * 开环运行时，反电势主电压由速度模型给出；PI 只补偿电阻压降、采样误差和负载扰动。
     */
    fUdPi = FOC_PI_Run(&g_stPiId, g_stCtrl.f32IdRef, g_stMotor.f32Id);
    fUqPi = FOC_PI_Run(&g_stPiIq, g_stCtrl.f32IqRef, g_stMotor.f32Iq);
    fUqFf = FOC_GetBemfFeedForwardPu() + FOC_RS_PU * g_stCtrl.f32IqRef;

    g_stMotor.f32UdRef = fUdPi;
    g_stMotor.f32UqRef = fUqFf + fUqPi;

    fVoltLimit = FOC_GetDynamicVoltageLimitPu();
    flimit_vector(&g_stMotor.f32UdRef, &g_stMotor.f32UqRef, fVoltLimit);

    /*--- 第4步：反Park ---*/
    FOC_InvPark(g_stMotor.f32UdRef, g_stMotor.f32UqRef,
                g_stMotor.f32Theta,
                &g_stMotor.f32Valpha, &g_stMotor.f32Vbeta);

    /*--- 第5步：SVPWM ---*/
    FOC_Svpwm(g_stMotor.f32Valpha, g_stMotor.f32Vbeta,
              &g_stMotor.u16Ta, &g_stMotor.u16Tb, &g_stMotor.u16Tc);

    /*--- 第6步：更新 TIM1 CCR ---*/
    TIM1->CCR1 = g_stMotor.u16Ta;
    TIM1->CCR2 = g_stMotor.u16Tb;
    TIM1->CCR3 = g_stMotor.u16Tc;

    g_stCtrl.u32RunFrames++;
}

/*============================================================================*/
/* ADC 电流采样 → pu                                                           */
/*============================================================================*/
void FOC_GetPhaseCurrent(void)
{
    uint16_t u16IaRaw, u16IbRaw;
    float fIa, fIb;

    /* 禁止 CH4 触发，防重入 */
    TIM1->CCER &= ~TIM_CCER_CC4E;

    u16IaRaw = (uint16_t)(ADC1->JDR1);
    u16IbRaw = (uint16_t)(ADC2->JDR1);

    /* current_pu = polarity * (raw - offset) / scale */
    fIa = FOC_CURRENT_POLARITY_A * ((float)u16IaRaw - FOC_fIaOffsetAdc) / FOC_ADC_CURRENT_SCALE;
    fIb = FOC_CURRENT_POLARITY_B * ((float)u16IbRaw - FOC_fIbOffsetAdc) / FOC_ADC_CURRENT_SCALE;

    g_stMotor.f32Ia = fclamp(fIa, -1.0f, 1.0f);
    g_stMotor.f32Ib = fclamp(fIb, -1.0f, 1.0f);
}
