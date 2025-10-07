/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file main.c
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
/**
 * @brief 主函数文件，包含LED控制示例
 * 
 * 本文件演示了如何在N32WB452芯片上控制LED灯：
 * - LED初始化
 * - LED开关控制
 * - LED闪烁效果
 */

#include "main.h"
#include <stdio.h>
#include <stdint.h>

/**
 * @brief  延时函数
 * 
 * 通过空循环实现简单的软件延时
 * 
 * @param count 延时循环计数，数值越大延时越长
 */
void Delay(uint32_t count)
{
    for (; count > 0; count--)
        ;
}

/**
 * @brief  初始化LED GPIO引脚
 * 
 * 配置指定GPIO引脚为推挽输出模式，用于控制LED
 * 
 * @param GPIOx GPIO端口，可选GPIOA~GPIOE
 * @param Pin GPIO引脚号，可选GPIO_PIN_0~GPIO_PIN_15
 */
void LedInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;

    /* 检查参数有效性 */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    /* 使能对应GPIO端口的时钟 */
    if (GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    }
    else if (GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    }
    else if (GPIOx == GPIOC)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);
    }
    else if (GPIOx == GPIOD)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOD, ENABLE);
    }
    else if (GPIOx == GPIOE)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOE, ENABLE);
    }

    /* 配置GPIO引脚 */
    if (Pin <= GPIO_PIN_ALL)
    {
        GPIO_InitStructure.Pin        = Pin;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     /* 推挽输出模式 */
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     /* 输出速度50MHz */
        GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);
    }
}

/**
 * @brief  点亮指定LED
 * 
 * 通过设置PBSC寄存器将指定引脚输出高电平，点亮LED
 * 
 * @param GPIOx GPIO端口
 * @param Pin GPIO引脚号
 */
void LedOn(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->PBSC = Pin;
}

/**
 * @brief  熄灭指定LED
 * 
 * 通过设置PBC寄存器将指定引脚输出低电平，熄灭LED
 * 
 * @param GPIOx GPIO端口
 * @param Pin GPIO引脚号
 */
void LedOff(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->PBC = Pin;
}

/**
 * @brief  控制LED开关状态
 * 
 * 根据参数的高16位和低16位分别控制LED的关和开
 * 
 * @param GPIOx GPIO端口
 * @param Pin 控制参数，低16位表示要点亮的引脚，高16位表示要熄灭的引脚
 */
void LedOnOff(GPIO_Module* GPIOx, uint32_t Pin)
{
    GPIOx->PBSC = Pin;
}

/**
 * @brief  LED翻转（闪烁）
 * 
 * 使用异或操作翻转指定引脚的电平状态，实现LED闪烁效果
 * 
 * @param GPIOx GPIO端口
 * @param Pin GPIO引脚号
 */
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->POD ^= Pin;
}

/**
 * @brief 断言失败处理函数
 * 
 * 当启用断言检查且断言条件不满足时，会调用此函数
 * 
 * @param expr 失败的断言表达式
 * @param file 发生失败的源文件名
 * @param line 发生失败的行号
 */
#ifdef USE_FULL_ASSERT
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    while (1)
    {
    }
}
#endif // USE_FULL_ASSERT

/**
 * @brief  主程序入口
 * 
 * 程序启动后首先初始化LED，然后进入循环执行LED控制逻辑
 */
int main(void)
{
    /* SystemInit()函数已在启动文件startup_n32wb452.s中调用 */

    /* 初始化Led1~Led5为输出推挽模式 */
    LedInit(PORT_GROUP1, LED1_PIN);
    LedInit(PORT_GROUP2, LED2_PIN);

    /* 点亮Led1 */
    LedOn(PORT_GROUP1, LED1_PIN);

    while (1)
    {
        /* LED1和LED2在同一个端口组，通过异或操作使Led2闪烁，不影响Led1 */
        LedBlink(PORT_GROUP1, LED1_PIN);

        /* LED3、LED4和LED5在同一个端口组 */
        /* 通过PBC寄存器关闭Led3和Led4，不影响同一端口组的其他引脚 */
        LedOff(PORT_GROUP2, LED2_PIN);
        /* 插入延时 */
        Delay(0x28FFFF);


        LedOnOff(PORT_GROUP2, (LED2_PIN << 16));
        /* 插入延时 */
        Delay(0x28FFFF);

        /* 点亮Led3 */
        LedOn(PORT_GROUP2, LED2_PIN);
        /* 插入延时 */
        Delay(0x28FFFF);
    }
}
/**
 * @}
 */