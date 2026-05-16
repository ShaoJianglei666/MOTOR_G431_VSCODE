/**
  ******************************************************************************
  * @file    key.h
  * @brief   按键模块 — 消抖 + 按下/释放事件检测
  ******************************************************************************
  */

#ifndef __KEY_H
#define __KEY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/** @brief 消抖时间 (ms) */
#define KEY_DEBOUNCE_MS  30U

/* Exported types ------------------------------------------------------------*/

/** @brief 按键 ID */
typedef enum
{
    KEY_ID_RUN  = 0,
    KEY_ID_COUNT
} Key_Id;

/** @brief 按键事件 */
typedef enum
{
    KEY_EVENT_NONE    = 0,   /**< 无事件 */
    KEY_EVENT_PRESS   = 1,   /**< 按下（下降沿消抖确认） */
    KEY_EVENT_RELEASE = 2    /**< 释放（上升沿消抖确认） */
} Key_Event;

/* Exported function prototypes ----------------------------------------------*/

/**
  * @brief  按键扫描（需在主循环中周期性调用，建议 ≥100Hz）
  */
void KEY_Scan(void);

/**
  * @brief  获取并清除按键事件
  * @param  eKey  按键 ID
  * @retval 事件类型（读取后自动清除）
  */
Key_Event KEY_GetEvent(Key_Id eKey);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H */
