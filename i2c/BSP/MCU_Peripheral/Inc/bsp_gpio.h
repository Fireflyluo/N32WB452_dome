/*-----------------------------------------------File Info------------------------------------------------
** File Name:               bsp_gpio.h  
** Last modified date:      2025.7.1
** Last version:            V0.1
** Description:             GPIO驱动接口声明
**
**--------------------------------------------------------------------------------------------------------            
** Created date:            2025.7.1
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            
**                          
**                          
**--------------------------------------------------------------------------------------------------------*/
#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#include "n32wb452.h"
#include "n32wb452_gpio.h"

#include "ioconfig.h"

typedef enum
{
  GPIO_PIN_RESET = 0U,
  GPIO_PIN_SET
} GPIO_PinState;

// 函数声明
void BSP_GPIO_Init(void);
void BSP_GPIO_DeInit(GPIO_Module  *GPIOx, uint32_t GPIO_Pin);

GPIO_PinState BSP_GPIO_ReadPin(GPIO_Module *GPIOx, uint16_t GPIO_Pin);
void BSP_GPIO_WritePin(GPIO_Module *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void BSP_GPIO_TogglePin(GPIO_Module *GPIOx, uint16_t GPIO_Pin);



#endif /* __BSP_GPIO_H__ */