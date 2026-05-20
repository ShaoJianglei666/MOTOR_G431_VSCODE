/**
  ******************************************************************************
  * @file    Motor_Param.h
  * @brief   电机参数配置文件
  *          所有电机相关的电气与机械参数集中定义于此文件。
  *          更换电机时只需修改此文件中的宏定义即可。
  ******************************************************************************
  * @attention
  *
  * 以下参数对应所使用的电机：
 *   - 极对数: 2
  *   - 额定电压: 24V
  *   - 额定转矩: 0.18 Nm
  *   - 相电阻: 0.575 Ω
  *   - 相电感: 1.05 mH
  *   - 反电势系数: 5.96708 V_peak L-L / krpm
  *
  ******************************************************************************
  */

#ifndef __MOTOR_PARAM_H
#define __MOTOR_PARAM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/**
  * @brief 电机极对数
  *        电气角度 = 机械角度 × MOTOR_POLE_PAIRS
  */
#define MOTOR_POLE_PAIRS               4U

/**
  * @brief 直流母线电压 (V)
  */
#define MOTOR_BUS_VOLTAGE              60.0f

/**
  * @brief 额定转矩 (Nm)
  */
#define MOTOR_RATED_TORQUE_NM          6.45f

/**
  * @brief 相电阻 (Ω)
  */
#define MOTOR_PHASE_RESISTANCE         0.006f

/**
  * @brief 相电感 (H)
  */
#define MOTOR_PHASE_INDUCTANCE         0.000035f

/**
  * @brief 反电势系数 (V_peak L-L / krpm)
  *        线线反电势峰值 / 机械转速 (kRPM)
  */
#define MOTOR_BEMF_CONST_V_LL          8.6961f

/**
  * @brief 转动惯量 (kg·m²)
  */
#define MOTOR_INERTIA_KGM2             0.0046f

/**
  * @brief 阻尼系数 (N·m·s)
  */
#define MOTOR_DAMPING_NMS              0.0000915f

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_PARAM_H */
