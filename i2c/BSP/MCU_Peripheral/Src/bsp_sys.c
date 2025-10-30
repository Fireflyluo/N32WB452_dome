/*-----------------------------------------------File Info------------------------------------------------
** File Name:               bsp_sys.c
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
#include "bsp_sys.h"

/* 全局变量定义 */
static volatile uint32_t system_tick = 0; /* 系统时钟计数器，每毫秒递增 */
static bool system_clock_ready = false;   /* 系统时钟初始化标志 */

/**
 * @brief 初始化系统滴答定时器（SysTick）
 * @param ticks_per_ms: 每毫秒的滴答数（基于系统时钟频率）
 * @retval true-成功, false-失败
 */
bool BSP_SysTick_Init(uint32_t ticks_per_ms)
{
    /* 参数检查 */
    if (ticks_per_ms == 0 || ticks_per_ms > SysTick_LOAD_RELOAD_Msk)
    {
        return false;
    }

    /* 配置SysTick */
    SysTick->LOAD = (ticks_per_ms & SysTick_LOAD_RELOAD_Msk) - 1; /* 重装载值 */
    SysTick->VAL = 0;                                             /* 清空当前值 */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |                  /* 使用处理器时钟 */
                    SysTick_CTRL_TICKINT_Msk |                    /* 启用中断 */
                    SysTick_CTRL_ENABLE_Msk;                      /* 启动定时器 */

    /* 设置SysTick中断优先级（较低优先级） */
    NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);

    system_clock_ready = true;
    return true;
}

/**
 * @brief SysTick中断服务函数
 * @note 每毫秒自动调用一次
 */
void SysTick_Handler(void)
{
    system_tick++;
}

/**
 * @brief 获取系统运行时间（毫秒）
 * @retval 从系统启动至今的毫秒数
 * @note 此函数可在中断中安全调用
 */
uint32_t BSP_GetTick(void)
{
    return system_tick;
}

/**
 * @brief 毫秒级延时函数
 * @param delay_ms: 需要延时的毫秒数
 * @note 阻塞式延时，期间CPU处于忙等待状态
 */
void BSP_Delay(uint32_t delay_ms)
{
    /* 检查系统时钟是否就绪 */
    if (!system_clock_ready)
    {
        return;
    }

    uint32_t start_tick = BSP_GetTick();
    uint32_t wait = delay_ms;

    /* 处理计数器溢出情况 */
    if (wait < BSP_MAX_DELAY)
    {
        wait += 1; /* 至少等待1个tick */
    }

    /* 忙等待循环（考虑计数器溢出） */
    while ((BSP_GetTick() - start_tick) < wait)
    {
        /* 可选：插入WFI指令降低功耗 */
        // __WFI();
    }
}
/**
 * @brief 微秒级精确延时
 * @param delay_us: 需要延时的微秒数
 * @note 基于指令周期的忙等待，适用于短时间精确延时
 */
void HAL_DelayUs(uint32_t delay_us)
{
    /* 基于系统时钟频率计算循环次数 */
    uint32_t cycles_per_us = SystemCoreClock / 1000000;
    uint32_t total_cycles = delay_us * cycles_per_us;

    /* 补偿函数调用开销（根据实际测量调整） */
    if (total_cycles > 10)
    {
        total_cycles -= 6; /* 经验值补偿 */
    }

    /* 空循环实现精确延时 */
    while (total_cycles--)
    {
        __NOP(); /* 无操作指令 */
    }
}

/**
 * @brief 获取系统时钟频率
 * @retval 当前系统核心时钟频率（Hz）
 */
uint32_t BSP_GetSystemClock(void)
{
    return SystemCoreClock;
}

/**
 * @brief 系统时钟就绪检查
 * @retval true-时钟就绪, false-时钟未就绪
 */
bool BSP_IsSystemClockReady(void)
{
    return system_clock_ready;
}

/**
 * @brief 获取从启动到现在的微秒数
 * @retval 微秒计数器（32位会溢出，注意使用场景）
 */
uint32_t BSP_GetTickUs(void)
{
    if (!system_clock_ready)
    {
        return 0;
    }

    /* 计算当前毫秒内的微秒数 */
    uint32_t current_ms = BSP_GetTick();
    uint32_t current_systick = SysTick->VAL;
    uint32_t ticks_per_ms = SysTick->LOAD + 1;

    /* 计算当前毫秒内已过去的微秒数 */
    uint32_t us_in_current_ms = ((ticks_per_ms - current_systick) * 1000) / ticks_per_ms;

    return (current_ms * 1000) + us_in_current_ms;
}