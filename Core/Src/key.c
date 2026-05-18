/**
  ******************************************************************************
  * @file    key.c
  * @brief   按键模块实现 — 消抖状态机 + 按下/释放事件
  *          外部上拉，按下 = 低电平 (GPIO_PIN_RESET)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "key.h"
#include "gpio.h"
#include "main.h"

/* Private types -------------------------------------------------------------*/

/** @brief 单个按键消抖状态 */
typedef enum
{
    KEY_ST_IDLE      = 0,   /**< 空闲，等待电平变化 */
    KEY_ST_DEBOUNCE  = 1    /**< 消抖中，计时等待 */
} Key_DebounceState;

/** @brief 单个按键上下文 */
typedef struct
{
    GPIO_TypeDef       *port;          /**< GPIO 端口 */
    uint16_t            pin;           /**< GPIO 引脚 */
    Key_DebounceState   state;         /**< 消抖状态 */
    uint8_t             lastRaw;       /**< 上一次原始电平 (0/1) */
    uint8_t             stableLevel;   /**< 消抖后的稳定电平 (0=按下, 1=释放) */
    uint32_t            tickStart;     /**< 消抖开始时刻 (ms) */
    Key_Event           event;         /**< 待读取的事件（锁存） */
} Key_Ctx;

/* Private variables ---------------------------------------------------------*/

static Key_Ctx s_astKeys[KEY_ID_COUNT] =
{
    [KEY_ID_RUN]      = { KEY_RUN_PORT,      KEY_RUN_PIN,      KEY_ST_IDLE, 1, 1, 0, KEY_EVENT_NONE },
    [KEY_ID_SPEED_UP] = { KEY_SPEED_UP_PORT, KEY_SPEED_UP_PIN, KEY_ST_IDLE, 1, 1, 0, KEY_EVENT_NONE },
};

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  按键扫描（主循环调用，建议 ≥100Hz）
  *         对每个按键执行消抖状态机，检测按下/释放事件。
  */
void KEY_Scan(void)
{
    uint32_t u32Now = HAL_GetTick();

    for (int i = 0; i < KEY_ID_COUNT; i++)
    {
        Key_Ctx *pCtx = &s_astKeys[i];
        uint8_t u8Raw = (HAL_GPIO_ReadPin(pCtx->port, pCtx->pin) == GPIO_PIN_RESET) ? 0 : 1;

        switch (pCtx->state)
        {
        case KEY_ST_IDLE:
            if (u8Raw != pCtx->stableLevel)
            {
                /* 电平变化 → 进入消抖 */
                pCtx->state     = KEY_ST_DEBOUNCE;
                pCtx->lastRaw   = u8Raw;
                pCtx->tickStart = u32Now;
            }
            break;

        case KEY_ST_DEBOUNCE:
            if (u8Raw != pCtx->lastRaw)
            {
                /* 消抖期间又变了 → 重新计时 */
                pCtx->lastRaw   = u8Raw;
                pCtx->tickStart = u32Now;
            }
            else if ((u32Now - pCtx->tickStart) >= KEY_DEBOUNCE_MS)
            {
                /* 消抖通过 → 确认电平变化 */
                uint8_t u8Old = pCtx->stableLevel;
                pCtx->stableLevel = u8Raw;
                pCtx->state       = KEY_ST_IDLE;

                /* 产生事件（锁存，不覆盖未读事件） */
                if (pCtx->event == KEY_EVENT_NONE)
                {
                    if (u8Old == 1 && u8Raw == 0)
                        pCtx->event = KEY_EVENT_PRESS;    /* 下降沿 */
                    else if (u8Old == 0 && u8Raw == 1)
                        pCtx->event = KEY_EVENT_RELEASE;  /* 上升沿 */
                }
            }
            break;
        }
    }
}

/**
  * @brief  获取并清除按键事件
  * @param  eKey  按键 ID
  * @retval 事件类型（读取后自动清除为 KEY_EVENT_NONE）
  */
Key_Event KEY_GetEvent(Key_Id eKey)
{
    if (eKey >= KEY_ID_COUNT) return KEY_EVENT_NONE;

    Key_Event eEvt = s_astKeys[eKey].event;
    s_astKeys[eKey].event = KEY_EVENT_NONE;
    return eEvt;
}
