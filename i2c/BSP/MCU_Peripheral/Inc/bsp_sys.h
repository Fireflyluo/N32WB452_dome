/*-----------------------------------------------File Info------------------------------------------------
** File Name:               bsp_sys.h
** Last modified date:      2025.10.30
** Last version:            V0.1
** Description:             BSP层系统级功能
**
**--------------------------------------------------------------------------------------------------------
** Created date:            2025.7.1
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:
**                          系统时钟配置相关函数
**
**--------------------------------------------------------------------------------------------------------*/
#ifndef __BSP_SYS_H__
#define __BSP_SYS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "n32wb452.h"

/* 常量定义 */
#define BSP_MAX_DELAY 0xFFFFFFFFU


    /* 外部变量声明（在system_n32wb452.c中定义） */
    extern uint32_t SystemCoreClock;

    /* 函数声明 */
    bool BSP_SysTick_Init(uint32_t ticks_per_ms);
    uint32_t BSP_GetTick(void);
    void BSP_Delay(uint32_t delay_ms);
    void BSP_DelayUs(uint32_t delay_us);
    uint32_t BSP_GetSystemClock(void);
    bool BSP_IsSystemClockReady(void);
    uint32_t BSP_GetTickUs(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SYS_H */