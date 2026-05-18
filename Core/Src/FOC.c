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

static inline float fwrap_pi(float theta)
{
    while (theta >= FOC_MATH_PI) theta -= FOC_2PI;
    while (theta < -FOC_MATH_PI) theta += FOC_2PI;
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
FOC_PI           g_stPiSpeed;
FOC_Luenberger   g_stLuenberger;
float            FOC_fIaOffsetAdc = 1972.0f;
float            FOC_fIbOffsetAdc = 1972.0f;

static float s_fObsIalphaPrev = 0.0f;
static float s_fObsIbetaPrev  = 0.0f;
static float s_fObsOmegaElec  = 0.0f;

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
    float fDelta;
    float fOut;
    float fKiTerm;

    pstPi->fProportional = pstPi->fKp * fErr;
    fDelta = pstPi->fKp * (fErr - pstPi->fErrPrev);
    pstPi->fErrPrev = fErr;

    fKiTerm = pstPi->fKi * fErr;

    if (!((pstPi->fOutPrev >= pstPi->fOutMax && fKiTerm > 0.0f) ||
          (pstPi->fOutPrev <= pstPi->fOutMin && fKiTerm < 0.0f)))
    {
        fDelta += fKiTerm;
        pstPi->fIntegral += fKiTerm;
        pstPi->fIntegral = fclamp(pstPi->fIntegral,
                                  pstPi->fOutMin * 0.5f,
                                  pstPi->fOutMax * 0.5f);
    }

    fOut = pstPi->fOutPrev + fDelta;
    fOut = fclamp(fOut, pstPi->fOutMin, pstPi->fOutMax);
    pstPi->fOutPrev = fOut;

    return fOut;
}

static float FOC_SpeedPI_Run(FOC_PI *pstPi, float fRefRpm, float fFbRpm)
{
    float fErr = fRefRpm - fFbRpm;
    float fOut;
    float fIntDelta;
    float fStep;

    if ((fErr < FOC_PI_SPEED_ERR_DEADBAND_RPM) &&
        (fErr > -FOC_PI_SPEED_ERR_DEADBAND_RPM))
    {
        fErr = 0.0f;
    }

    pstPi->fProportional = pstPi->fKp * fErr;

    fOut = pstPi->fProportional + pstPi->fIntegral;
    fIntDelta = pstPi->fKi * fErr;

    if (!((fOut >= pstPi->fOutMax && fIntDelta > 0.0f) ||
          (fOut <= pstPi->fOutMin && fIntDelta < 0.0f)))
    {
        pstPi->fIntegral += fIntDelta;
    }

    if ((fErr < 0.0f) && (pstPi->fIntegral > 0.0f))
    {
        pstPi->fIntegral += FOC_PI_SPEED_UNWIND_KP * fErr;
    }
    else if ((fErr > 0.0f) && (pstPi->fIntegral < 0.0f))
    {
        pstPi->fIntegral += FOC_PI_SPEED_UNWIND_KP * fErr;
    }

    pstPi->fIntegral = fclamp(pstPi->fIntegral,
                              pstPi->fOutMin,
                              pstPi->fOutMax);

    fOut = pstPi->fProportional + pstPi->fIntegral;
    fOut = fclamp(fOut, pstPi->fOutMin, pstPi->fOutMax);

    fStep = fOut - pstPi->fOutPrev;
    if (fStep > FOC_PI_SPEED_SLEW_UP_Q12)
    {
        fOut = pstPi->fOutPrev + FOC_PI_SPEED_SLEW_UP_Q12;
    }
    else if (fStep < -FOC_PI_SPEED_SLEW_DOWN_Q12)
    {
        fOut = pstPi->fOutPrev - FOC_PI_SPEED_SLEW_DOWN_Q12;
    }

    fOut = fclamp(fOut, pstPi->fOutMin, pstPi->fOutMax);
    pstPi->fOutPrev = fOut;
    pstPi->fErrPrev = fErr;
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
    g_stCtrl.u16DiagStage = FOC_STAGE_STOP;
    FOC_Luenberger_Init();

    /* Id PI */
    g_stPiId.fKp       = FOC_PI_ID_KP;
    g_stPiId.fKi       = FOC_PI_ID_KI;
    g_stPiId.fErrPrev  = 0.0f;
    g_stPiId.fOutPrev  = 0.0f;
    g_stPiId.fIntegral = 0.0f;
    g_stPiId.fProportional = 0.0f;
    g_stPiId.fOutMax   =  FOC_PI_CORRECTION_MAX_PU;
    g_stPiId.fOutMin   = -FOC_PI_CORRECTION_MAX_PU;

    /* Iq PI */
    g_stPiIq.fKp       = FOC_PI_IQ_KP;
    g_stPiIq.fKi       = FOC_PI_IQ_KI;
    g_stPiIq.fErrPrev  = 0.0f;
    g_stPiIq.fOutPrev  = 0.0f;
    g_stPiIq.fIntegral = 0.0f;
    g_stPiIq.fProportional = 0.0f;
    g_stPiIq.fOutMax   =  FOC_PI_CORRECTION_MAX_PU;
    g_stPiIq.fOutMin   = -FOC_PI_CORRECTION_MAX_PU;

    /* Speed PI: output is extra Iq on top of the closed-loop base current. */
    g_stPiSpeed.fKp       = FOC_PI_SPEED_KP;
    g_stPiSpeed.fKi       = FOC_PI_SPEED_KI;
    g_stPiSpeed.fErrPrev  = 0.0f;
    g_stPiSpeed.fOutPrev  = 0.0f;
    g_stPiSpeed.fIntegral = 0.0f;
    g_stPiSpeed.fProportional = 0.0f;
    g_stPiSpeed.fOutMax   =  FOC_PI_SPEED_OUT_MAX;
    g_stPiSpeed.fOutMin   =  FOC_PI_SPEED_OUT_MIN;
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
        g_stCtrl.u16DiagStage = FOC_STAGE_STOP;
        /* 目标转速 > 0 → 进入 IF 开环 */
        if (g_stCtrl.f32TargetRpm > 0.0f)
        {
            g_stCtrl.f32RpmRamp   = 0.0f;
            g_stCtrl.u32RampCount = 0;
            g_stCtrl.u32RunFrames = 0;
            g_stCtrl.u32SettleFrames = 0;
            g_stCtrl.u32SpeedStableCount = 0;
            g_stCtrl.u32BlendCount = 0;
            g_stCtrl.u32SwitchStableCount = 0;
            g_stCtrl.u32ObsLostCount = 0;
            g_stCtrl.f32RampedTargetRpm = 0.0f;
            g_stCtrl.f32SpdIntegral = 0.0f;
            g_stCtrl.f32SpdProportional = 0.0f;
            g_stCtrl.f32IqBase = 0.0f;
            g_stCtrl.f32IqBaseTarget = 0.0f;
            g_stCtrl.u16DiagTransitionFlag = 0;
            g_stCtrl.u16DiagStage = FOC_STAGE_ALIGN;
            g_stMotor.f32Theta    = 0.0f;

            g_stPiId.fErrPrev = 0.0f;
            g_stPiId.fOutPrev = 0.0f;
            g_stPiId.fIntegral = 0.0f;
            g_stPiId.fProportional = 0.0f;
            g_stPiId.fOutMax = FOC_PI_CORRECTION_MAX_PU;
            g_stPiId.fOutMin = -FOC_PI_CORRECTION_MAX_PU;
            g_stPiIq.fErrPrev = 0.0f;
            g_stPiIq.fOutPrev = 0.0f;
            g_stPiIq.fIntegral = 0.0f;
            g_stPiIq.fProportional = 0.0f;
            g_stPiIq.fOutMax = FOC_PI_CORRECTION_MAX_PU;
            g_stPiIq.fOutMin = -FOC_PI_CORRECTION_MAX_PU;
            g_stPiSpeed.fErrPrev = 0.0f;
            g_stPiSpeed.fOutPrev = 0.0f;
            g_stPiSpeed.fIntegral = 0.0f;
            g_stPiSpeed.fProportional = 0.0f;

            g_stCtrl.eMode = FOC_MODE_IF_OPENLOOP;
        }
        break;

    case FOC_MODE_IF_OPENLOOP:
    {
        /*--- 目标转速归零 → 回到 STOP ---*/
        if (g_stCtrl.f32TargetRpm == 0.0f)
        {
            g_stCtrl.eMode = FOC_MODE_STOP;
            g_stCtrl.u16DiagStage = FOC_STAGE_STOP;
            break;
        }

        /*--- 参考启动阶段：Iq 软启动，角度从一开始缓慢旋转 ---*/
        if (g_stCtrl.u32RampCount < FOC_IF_IQ_RAMP_FRAMES)
        {
            float fProgress = (float)g_stCtrl.u32RampCount / (float)FOC_IF_IQ_RAMP_FRAMES;
            g_stCtrl.f32IqRef = FOC_IF_IQ_START
                               + (FOC_IF_STARTUP_IQ - FOC_IF_IQ_START) * fProgress;
            g_stCtrl.u16DiagStage = FOC_STAGE_ALIGN;
        }
        else
        {
            g_stCtrl.f32IqRef = FOC_IF_STARTUP_IQ;
            g_stCtrl.u16DiagStage = FOC_STAGE_OPEN_LOOP;
        }
        g_stCtrl.f32IdRef = 0.0f;

#if FOC_FORCE_OPEN_LOOP
        if ((g_stCtrl.f32RpmRamp < g_stCtrl.f32TargetRpm) &&
            ((g_stCtrl.u32RampCount % FOC_IF_RAMP_INTERVAL) == 0U))
        {
            g_stCtrl.f32RpmRamp += FOC_IF_RAMP_STEP_RPM;
            if (g_stCtrl.f32RpmRamp > g_stCtrl.f32TargetRpm)
            {
                g_stCtrl.f32RpmRamp = g_stCtrl.f32TargetRpm;
            }
        }
#else
        if ((g_stCtrl.f32RpmRamp < FOC_IF_SWITCH_RPM) &&
            ((g_stCtrl.u32RampCount % FOC_IF_RAMP_INTERVAL) == 0U))
        {
            g_stCtrl.f32RpmRamp += FOC_IF_RAMP_STEP_RPM;
            if (g_stCtrl.f32RpmRamp > FOC_IF_SWITCH_RPM)
            {
                g_stCtrl.f32RpmRamp = FOC_IF_SWITCH_RPM;
            }
        }
#endif

        if (g_stCtrl.f32RpmRamp >= FOC_IF_SWITCH_RPM)
        {
            g_stCtrl.u32SettleFrames++;
        }

        fStep = g_stCtrl.f32RpmRamp * (FOC_2PI * (float)MOTOR_POLE_PAIRS / 60.0f) * FOC_CTRL_TS;
        g_stMotor.f32Theta = fwrap_2pi(g_stMotor.f32Theta + fStep);

#if !FOC_FORCE_OPEN_LOOP
        if (g_stCtrl.u32SettleFrames >= FOC_IF_SETTLE_FRAMES)
        {
            float fTol = FOC_IF_SWITCH_RPM * (float)FOC_IF_SPEED_TOL_PCT / 100.0f;
            uint8_t u8SpeedOk = ((g_stLuenberger.f32SpeedObs > (FOC_IF_SWITCH_RPM - fTol)) &&
                                 (g_stLuenberger.f32SpeedObs < (FOC_IF_SWITCH_RPM + fTol))) ? 1U : 0U;

            if (u8SpeedOk)
            {
                g_stCtrl.u32SpeedStableCount++;
                if (g_stCtrl.u32SpeedStableCount >= FOC_IF_STABLE_COUNT_THR)
                {
                    g_stCtrl.f32ThetaErrSave = fwrap_pi(g_stLuenberger.f32ThetaObs - g_stMotor.f32Theta);
                    g_stCtrl.f32ThetaIfRef = g_stMotor.f32Theta;
                    g_stCtrl.u32BlendCount = 0;
                    g_stCtrl.u16DiagTransitionFlag = 1U;
                    g_stCtrl.u16DiagStage = FOC_STAGE_TRANSITION;
                    g_stCtrl.eMode = FOC_MODE_TRANSITION;
                }
            }
            else
            {
                g_stCtrl.u32SpeedStableCount = 0;
            }
        }
#endif

        g_stCtrl.f32TorqueAngle = fwrap_2pi(g_stLuenberger.f32ThetaObs - g_stMotor.f32Theta);
        g_stCtrl.u32RampCount++;
        break;
    }

    case FOC_MODE_TRANSITION:
    {
        float fCnt;
        float fRemain;

        if (g_stCtrl.f32TargetRpm == 0.0f)
        {
            g_stCtrl.eMode = FOC_MODE_STOP;
            g_stCtrl.u16DiagStage = FOC_STAGE_STOP;
            break;
        }

        g_stCtrl.u16DiagStage = FOC_STAGE_TRANSITION;

        g_stCtrl.u32BlendCount++;
        fCnt = (float)g_stCtrl.u32BlendCount;
        if (fCnt > (float)FOC_TRANSITION_FRAMES)
        {
            fCnt = (float)FOC_TRANSITION_FRAMES;
        }

        fRemain = g_stCtrl.f32ThetaErrSave
                * ((float)FOC_TRANSITION_FRAMES - fCnt)
                / (float)FOC_TRANSITION_FRAMES;
        g_stMotor.f32Theta = fwrap_2pi(g_stLuenberger.f32ThetaObs - fRemain);
        g_stCtrl.f32TorqueAngle = fwrap_2pi(g_stLuenberger.f32ThetaObs - g_stMotor.f32Theta);

        g_stCtrl.f32IdRef = 0.0f;
        g_stCtrl.f32IqRef = FOC_IF_STARTUP_IQ;

        if (g_stCtrl.u32BlendCount >= FOC_TRANSITION_FRAMES)
        {
            float fIqPreload;

            g_stMotor.f32Theta = g_stLuenberger.f32ThetaObs;

            /*--- 预载速度 PI：对标 Q12 参考工程，根据转速比估算维持 Iq ---*/
            /* Q12 参考: i32IqEst = i16IqRef * FOC_CTRL_RPM_SWITCH / u16TargetRpm / 3 */
            fIqPreload = g_stCtrl.f32IqRef
                       * FOC_IF_SWITCH_RPM
                       / g_stCtrl.f32TargetRpm
                       * FOC_CLOSED_IQ_BASE_RATIO;
            if (fIqPreload > g_stCtrl.f32IqRef)
            {
                fIqPreload = g_stCtrl.f32IqRef;
            }
            if (fIqPreload < FOC_CLOSED_IQ_BASE_TARGET)
            {
                fIqPreload = FOC_CLOSED_IQ_BASE_TARGET;
            }

            /* 速度 PI 内部输出为 Q12 计数，进入电流环前再转为 pu。 */
            {
                float fCurSpd = g_stLuenberger.f32SpeedObs;
                if (fCurSpd < 0.0f)
                    fCurSpd = 0.0f;
                if (fCurSpd > g_stCtrl.f32TargetRpm)
                    fCurSpd = g_stCtrl.f32TargetRpm;
                g_stCtrl.f32RampedTargetRpm = fCurSpd;
            }

            g_stPiSpeed.fOutMax = FOC_PI_SPEED_OUT_MAX;
            g_stPiSpeed.fOutMin = FOC_PI_SPEED_OUT_MIN;
            g_stPiSpeed.fErrPrev = g_stCtrl.f32RampedTargetRpm - g_stLuenberger.f32SpeedObs;
            g_stPiSpeed.fOutPrev = FOC_PU_TO_Q12(fIqPreload);
            g_stPiSpeed.fIntegral = FOC_PU_TO_Q12(fIqPreload);
            g_stPiSpeed.fProportional = FOC_PI_SPEED_KP * g_stPiSpeed.fErrPrev;

            /* 不设置 f32IqRef：保持过渡阶段值，直到速度 PI 在下一 500Hz 周期自然接管输出 */
            /* 保存基准电流（仅诊断） */
            g_stCtrl.f32IqBase = fIqPreload;
            g_stCtrl.f32IqBaseTarget = FOC_CLOSED_IQ_BASE_TARGET;

            /* 预载电流环 PI，确保 Ud/Uq 连续 */
            g_stPiId.fOutMax = FOC_VOLTAGE_MAX_PU;
            g_stPiId.fOutMin = -FOC_VOLTAGE_MAX_PU;
            g_stPiIq.fOutMax = FOC_VOLTAGE_MAX_PU;
            g_stPiIq.fOutMin = -FOC_VOLTAGE_MAX_PU;

            g_stPiId.fErrPrev = g_stCtrl.f32IdRef - g_stMotor.f32Id;
            g_stPiId.fOutPrev = fclamp(g_stMotor.f32UdRef,
                                       g_stPiId.fOutMin,
                                       g_stPiId.fOutMax);
            g_stPiId.fIntegral = g_stPiId.fOutPrev;
            g_stPiId.fProportional = g_stPiId.fKp * g_stPiId.fErrPrev;

            g_stPiIq.fErrPrev = g_stCtrl.f32IqRef - g_stMotor.f32Iq;
            g_stPiIq.fOutPrev = fclamp(g_stMotor.f32UqRef,
                                       g_stPiIq.fOutMin,
                                       g_stPiIq.fOutMax);
            g_stPiIq.fIntegral = g_stPiIq.fOutPrev;
            g_stPiIq.fProportional = g_stPiIq.fKp * g_stPiIq.fErrPrev;

            g_stCtrl.f32SpdProportional = FOC_Q12_TO_PU(g_stPiSpeed.fProportional);
            g_stCtrl.f32SpdIntegral = FOC_Q12_TO_PU(g_stPiSpeed.fIntegral);

            g_stCtrl.eMode = FOC_MODE_CLOSED_LOOP;
            g_stCtrl.u16DiagTransitionFlag = 0U;
            g_stCtrl.u16DiagStage = FOC_STAGE_CLOSED_LOOP;
            g_stCtrl.u32SwitchStableCount = 0;
            g_stCtrl.u32ObsLostCount = 0;
        }
        break;
    }

    case FOC_MODE_CLOSED_LOOP:
        if (g_stCtrl.f32TargetRpm == 0.0f)
        {
            g_stCtrl.eMode = FOC_MODE_STOP;
            g_stCtrl.u16DiagStage = FOC_STAGE_STOP;
            break;
        }

        g_stCtrl.u16DiagStage = FOC_STAGE_CLOSED_LOOP;
        g_stMotor.f32Theta = g_stLuenberger.f32ThetaObs;
        g_stCtrl.f32IdRef = 0.0f;
        g_stCtrl.f32RpmRamp = g_stLuenberger.f32SpeedObs;

        if (g_stCtrl.u32SwitchStableCount < FOC_CLOSED_STABLE_FRAMES)
        {
            g_stCtrl.u32SwitchStableCount++;
        }
        else if (!g_stLuenberger.u16Locked)
        {
            g_stCtrl.u32ObsLostCount++;
            if (g_stCtrl.u32ObsLostCount >= FOC_OBS_LOST_THRESHOLD)
            {
                g_stCtrl.eMode = FOC_MODE_STOP;
                g_stCtrl.u16DiagStage = FOC_STAGE_STOP;
                g_stCtrl.f32TargetRpm = 0.0f;
                g_stCtrl.f32IqBase = 0.0f;
                g_stCtrl.f32IqBaseTarget = 0.0f;
                g_stPiId.fErrPrev = 0.0f;
                g_stPiId.fOutPrev = 0.0f;
                g_stPiId.fIntegral = 0.0f;
                g_stPiId.fProportional = 0.0f;
                g_stPiId.fOutMax = FOC_PI_CORRECTION_MAX_PU;
                g_stPiId.fOutMin = -FOC_PI_CORRECTION_MAX_PU;
                g_stPiIq.fErrPrev = 0.0f;
                g_stPiIq.fOutPrev = 0.0f;
                g_stPiIq.fIntegral = 0.0f;
                g_stPiIq.fProportional = 0.0f;
                g_stPiIq.fOutMax = FOC_PI_CORRECTION_MAX_PU;
                g_stPiIq.fOutMin = -FOC_PI_CORRECTION_MAX_PU;
                g_stPiSpeed.fErrPrev = 0.0f;
                g_stPiSpeed.fOutPrev = 0.0f;
                g_stPiSpeed.fIntegral = 0.0f;
                g_stPiSpeed.fProportional = 0.0f;
            }
        }
        else
        {
            g_stCtrl.u32ObsLostCount = 0;
        }
        break;

    case FOC_MODE_FAULT:
        g_stCtrl.u16DiagStage = FOC_STAGE_FAULT;
        break;
    }
}

/*============================================================================*/
/* 龙伯格反电势观测器 — 旁路诊断，不参与开环角度/电压控制                     */
/*============================================================================*/
void FOC_Luenberger_Init(void)
{
    memset(&g_stLuenberger, 0, sizeof(g_stLuenberger));
    s_fObsIalphaPrev = 0.0f;
    s_fObsIbetaPrev  = 0.0f;
    s_fObsOmegaElec  = 0.0f;
}

void FOC_Luenberger_Run(void)
{
    float fOmegaOpen;
    float fSpeedBlend;
    float fDiAlpha;
    float fDiBeta;
    float fEalpha;
    float fEbeta;
    float fSin;
    float fCos;
    float fErr;
    float fMagSq;

    fDiAlpha = g_stMotor.f32Ialpha - s_fObsIalphaPrev;
    fDiBeta  = g_stMotor.f32Ibeta  - s_fObsIbetaPrev;
    s_fObsIalphaPrev = g_stMotor.f32Ialpha;
    s_fObsIbetaPrev  = g_stMotor.f32Ibeta;

    fEalpha = g_stMotor.f32Valpha
            - FOC_RS_PU * g_stMotor.f32Ialpha
            - FOC_LS_OVER_TS_PU * fDiAlpha;
    fEbeta  = g_stMotor.f32Vbeta
            - FOC_RS_PU * g_stMotor.f32Ibeta
            - FOC_LS_OVER_TS_PU * fDiBeta;

    fEalpha = g_stLuenberger.f32Ealpha
            + FOC_OBS_LPF_GAIN * (fEalpha - g_stLuenberger.f32Ealpha);
    fEbeta  = g_stLuenberger.f32Ebeta
            + FOC_OBS_LPF_GAIN * (fEbeta - g_stLuenberger.f32Ebeta);

    g_stLuenberger.f32Ealpha = fclamp(fEalpha, -0.98f, 0.98f);
    g_stLuenberger.f32Ebeta  = fclamp(fEbeta,  -0.98f, 0.98f);

    f32_sincos_lut(g_stLuenberger.f32ThetaObs, &fSin, &fCos);
    fErr = -g_stLuenberger.f32Ealpha * fCos
           -g_stLuenberger.f32Ebeta  * fSin;
    fErr = fclamp(fErr, -FOC_OBS_PLL_ERR_MAX, FOC_OBS_PLL_ERR_MAX);

    fOmegaOpen = g_stCtrl.f32RpmRamp * FOC_2PI * (float)MOTOR_POLE_PAIRS / 60.0f;
    fOmegaOpen = fclamp(fOmegaOpen, 0.0f, FOC_OBS_MAX_OMEGA_ELEC);
    fSpeedBlend = (g_stCtrl.eMode == FOC_MODE_CLOSED_LOOP) ? 0.0f : FOC_OBS_PLL_SPEED_BLEND;

    s_fObsOmegaElec += fSpeedBlend * (fOmegaOpen - s_fObsOmegaElec)
                     + FOC_OBS_PLL_KI * fErr;
    s_fObsOmegaElec = fclamp(s_fObsOmegaElec, 0.0f, FOC_OBS_MAX_OMEGA_ELEC);

    g_stLuenberger.f32ThetaObs = fwrap_2pi(g_stLuenberger.f32ThetaObs
                                        + s_fObsOmegaElec * FOC_CTRL_TS
                                        + FOC_OBS_PLL_KP * fErr);
    g_stLuenberger.f32SpeedObs = s_fObsOmegaElec * FOC_ELEC_OMEGA_TO_RPM;
    g_stLuenberger.f32ErrPll   = fErr;

    {
        float a = g_stLuenberger.f32Ealpha;
        float b = g_stLuenberger.f32Ebeta;
        if (a < 0.0f) a = -a;
        if (b < 0.0f) b = -b;
        if (a < b)
        {
            float t = a;
            a = b;
            b = t;
        }
        g_stLuenberger.f32BemfMag = a + 0.5f * b;
    }

    fMagSq = g_stLuenberger.f32Ealpha * g_stLuenberger.f32Ealpha
           + g_stLuenberger.f32Ebeta  * g_stLuenberger.f32Ebeta;
    g_stLuenberger.u16Locked = ((fMagSq > FOC_OBS_LOCK_BEMF_SQ_THR) &&
                                (g_stLuenberger.f32SpeedObs > FOC_OBS_LOCK_SPEED_RPM)) ? 1U : 0U;
}

/*============================================================================*/
/* FOC 控制步进 — 10kHz ADC 中断中调用                                         */
/*============================================================================*/
void FOC_ControlStep(void)
{
#if !FOC_FORCE_OPEN_LOOP
    static uint16_t s_u16SpeedLoopCnt = 0U;
    static uint16_t s_u16SpeedRampDiv = 0U;
#endif
    float fUdPi;
    float fUqPi;
    float fUqFf;
    float fVoltLimit;

    /*--- 第0步：状态机 ---*/
    FOC_StateMachine_Run();

    /*--- STOP：输出 50% 占空比 ---*/
    if ((g_stCtrl.eMode == FOC_MODE_STOP) || (g_stCtrl.eMode == FOC_MODE_FAULT))
    {
#if !FOC_FORCE_OPEN_LOOP
        s_u16SpeedLoopCnt = 0U;
        s_u16SpeedRampDiv = 0U;
#endif
        TIM1->CCR1 = PWM_HALF_CYCLE;
        TIM1->CCR2 = PWM_HALF_CYCLE;
        TIM1->CCR3 = PWM_HALF_CYCLE;
        return;
    }

    /*--- 闭环速度 PI：Q12输出转pu后作为 IqRef，500Hz 分频 ---*/
#if !FOC_FORCE_OPEN_LOOP
    if (g_stCtrl.eMode == FOC_MODE_CLOSED_LOOP)
    {
        s_u16SpeedLoopCnt++;
        if (s_u16SpeedLoopCnt >= FOC_SPEED_LOOP_DECIMATION)
        {
            s_u16SpeedLoopCnt = 0U;

            /*--- 目标转速斜坡 ---*/
            s_u16SpeedRampDiv++;
            if (s_u16SpeedRampDiv >= FOC_SPEED_RAMP_DIV)
            {
                s_u16SpeedRampDiv = 0U;
                if (g_stCtrl.f32RampedTargetRpm < g_stCtrl.f32TargetRpm)
                {
                    g_stCtrl.f32RampedTargetRpm += FOC_SPEED_RAMP_STEP_RPM;
                    if (g_stCtrl.f32RampedTargetRpm > g_stCtrl.f32TargetRpm)
                        g_stCtrl.f32RampedTargetRpm = g_stCtrl.f32TargetRpm;
                }
                else if (g_stCtrl.f32RampedTargetRpm > g_stCtrl.f32TargetRpm)
                {
                    g_stCtrl.f32RampedTargetRpm -= FOC_SPEED_RAMP_STEP_RPM;
                    if (g_stCtrl.f32RampedTargetRpm < g_stCtrl.f32TargetRpm)
                        g_stCtrl.f32RampedTargetRpm = g_stCtrl.f32TargetRpm;
                }
            }

            /* 速度 PI 输出为 Q12 计数，换算为 pu 后再作为电流环 IqRef。 */
            g_stPiSpeed.fOutMax = FOC_PI_SPEED_OUT_MAX;
            g_stPiSpeed.fOutMin = FOC_PI_SPEED_OUT_MIN;
            g_stCtrl.f32IqRef = FOC_Q12_TO_PU(FOC_SpeedPI_Run(&g_stPiSpeed,
                                                              g_stCtrl.f32RampedTargetRpm,
                                                              g_stLuenberger.f32SpeedObs));//这里不用除以4096
            g_stCtrl.f32IqRef = fclamp(g_stCtrl.f32IqRef,
                                       -FOC_CLOSED_IQ_REF_MAX,
                                        FOC_CLOSED_IQ_REF_MAX);

            /* 诊断输出 */
            g_stCtrl.f32SpdProportional = FOC_Q12_TO_PU(g_stPiSpeed.fProportional);
            g_stCtrl.f32SpdIntegral     = FOC_Q12_TO_PU(g_stPiSpeed.fIntegral);
        }
    }
    else
    {
        s_u16SpeedLoopCnt = 0U;
        s_u16SpeedRampDiv = 0U;
    }
#endif

    /*--- 第1步：Clarke ---*/
    FOC_Clarke(g_stMotor.f32Ia, g_stMotor.f32Ib,
               &g_stMotor.f32Ialpha, &g_stMotor.f32Ibeta);

    /*--- 第2步：Park ---*/
    FOC_Park(g_stMotor.f32Ialpha, g_stMotor.f32Ibeta,
             g_stMotor.f32Theta, &g_stMotor.f32Id, &g_stMotor.f32Iq);

    if (g_stCtrl.eMode == FOC_MODE_CLOSED_LOOP)
    {
        /* 闭环后电流 PI 直接输出完整 Ud/Uq，严格对应参考工程电流环。 */
        g_stMotor.f32UdRef = FOC_PI_Run(&g_stPiId, g_stCtrl.f32IdRef, g_stMotor.f32Id);
        g_stMotor.f32UqRef = FOC_PI_Run(&g_stPiIq, g_stCtrl.f32IqRef, g_stMotor.f32Iq);
        flimit_vector(&g_stMotor.f32UdRef, &g_stMotor.f32UqRef, FOC_VOLTAGE_MAX_PU);
    }
    else
    {
        /* 开环阶段：速度电压前馈 + 小幅电流修正。 */
        fUdPi = FOC_PI_Run(&g_stPiId, g_stCtrl.f32IdRef, g_stMotor.f32Id);
        fUqPi = FOC_PI_Run(&g_stPiIq, g_stCtrl.f32IqRef, g_stMotor.f32Iq);
        fUqFf = FOC_GetBemfFeedForwardPu() + FOC_RS_PU * g_stCtrl.f32IqRef;

        g_stMotor.f32UdRef = fUdPi;
        g_stMotor.f32UqRef = fUqFf + fUqPi;

        fVoltLimit = FOC_GetDynamicVoltageLimitPu();
        flimit_vector(&g_stMotor.f32UdRef, &g_stMotor.f32UqRef, fVoltLimit);
    }

    /*--- 第4步：反Park ---*/
    FOC_InvPark(g_stMotor.f32UdRef, g_stMotor.f32UqRef,
                g_stMotor.f32Theta,
                &g_stMotor.f32Valpha, &g_stMotor.f32Vbeta);

    /*--- 旁路观测器：只更新观测角度，不参与控制 ---*/
    FOC_Luenberger_Run();

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

    // /* 禁止 CH4 触发，防重入 */
    // TIM1->CCER &= ~TIM_CCER_CC4E;这里是个bug，不注释的话会第二次启动失败，因为adc中断被关了

    u16IaRaw = (uint16_t)(ADC1->JDR1);
    u16IbRaw = (uint16_t)(ADC2->JDR1);

    /* 验证版硬件电流方向为 offset - raw，这里用 polarity 保持坐标系一致。 */
    fIa = FOC_CURRENT_POLARITY_A * ((float)u16IaRaw - FOC_fIaOffsetAdc) / FOC_ADC_CURRENT_SCALE;
    fIb = FOC_CURRENT_POLARITY_B * ((float)u16IbRaw - FOC_fIbOffsetAdc) / FOC_ADC_CURRENT_SCALE;

    g_stMotor.f32Ia = fclamp(fIa, -1.0f, 1.0f);
    g_stMotor.f32Ib = fclamp(fIb, -1.0f, 1.0f);
}
