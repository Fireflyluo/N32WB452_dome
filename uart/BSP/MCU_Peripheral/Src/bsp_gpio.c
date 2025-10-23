/*-----------------------------------------------File Info------------------------------------------------
** File Name:               bsp_gpio.c  
** Last modified date:      2025.10.19
** Last version:            V0.1
** Description:             BSP层GPIO设备初始化代码
**
**--------------------------------------------------------------------------------------------------------            
** Created date:            2025.7.1
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            
**                          定义GPIO初始化函数的实现；                      
**--------------------------------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "bsp_gpio.h"

void BSP_GPIO_Init(void)
{
    GPIO_InitType GPIO_InitStructure;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);

    // 配置LED引脚
    GPIO_InitStructure.Pin        = R_LED_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     /* 推挽输出模式 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     /* 输出速度50MHz */
    GPIO_InitPeripheral(R_LED_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.Pin        = G_LED_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     /* 推挽输出模式 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     /* 输出速度50MHz */
    GPIO_InitPeripheral(G_LED_GPIO, &GPIO_InitStructure);

    // 电路切换引脚
    GPIO_InitStructure.Pin        = CIRCUIT_SWITCH_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(CIRCUIT_SWITCH_GPIO, &GPIO_InitStructure);

    // 配置屏幕引脚
    GPIO_InitStructure.Pin        = SRC_CS_PIN | SRC_DC_PIN | SRC_RES_PIN ;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     /* 推挽输出模式 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     /* 输出速度50MHz */
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    // 配置SPI2片选引脚
    GPIO_InitStructure.Pin        = SPI2_NSS_PIN;          /* SPI2片选引脚 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     /* 推挽输出模式 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     /* 输出速度50MHz */
    GPIO_InitPeripheral(SPI2_NSS_GPIO, &GPIO_InitStructure);


}

/**
 * @brief 复位GPIO引脚配置
 * @param GPIOx GPIO端口基地址
 * @param GPIO_Pin 要复位的引脚
 */
void BSP_GPIO_DeInit(GPIO_Module *GPIOx, uint32_t GPIO_Pin)
{
    // 检查参数有效性
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN(GPIO_Pin));
    
    // 通过逐个引脚复位来替代整个GPIO端口复位
    // 配置为默认的浮空输入模式
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);
}


/**
 * @brief 读取指定GPIO引脚的状态
 * @param GPIOx GPIO端口基地址
 * @param GPIO_Pin 要读取的引脚
 * @return 引脚状态(GPIO_PIN_SET 或 GPIO_PIN_RESET)
 */
GPIO_PinState BSP_GPIO_ReadPin(GPIO_Module *GPIOx, uint16_t GPIO_Pin)
{
    return (GPIO_PinState)GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
}

/**
 * @brief 设置或清除指定GPIO引脚的电平状态
 * @param GPIOx GPIO端口基地址
 * @param GPIO_Pin 要操作的引脚
 * @param PinState 引脚状态(GPIO_PIN_SET 或 GPIO_PIN_RESET)
 */
void BSP_GPIO_WritePin(GPIO_Module *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    GPIO_WriteBit(GPIOx, GPIO_Pin, (Bit_OperateType)PinState);
}

/**
 * @brief 切换指定GPIO引脚的电平状态
 * @param GPIOx GPIO端口基地址
 * @param GPIO_Pin 要切换的引脚
 */
void BSP_GPIO_TogglePin(GPIO_Module *GPIOx, uint16_t GPIO_Pin)
{
    // 读取当前引脚状态并切换
    if (GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin) != (uint8_t)Bit_RESET)
    {
        // 当前为高电平，设置为低电平
        GPIOx->PBC = GPIO_Pin;
    }
    else
    {
        // 当前为低电平，设置为高电平
        GPIOx->PBSC = GPIO_Pin;
    }
}