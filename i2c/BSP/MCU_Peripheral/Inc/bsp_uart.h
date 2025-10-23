/*-----------------------------------------------File Info------------------------------------------------
** File Name:               bsp_uart.h
** Last modified date:      2025.7.1
** Last version:            V0.1
** Description:             uart驱动接口声明
**
**--------------------------------------------------------------------------------------------------------
** Created date:            2025.7.1
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            定义uart编号、DMA模式枚举；
**                          声明uart初始化、发送/接收函数；
**
**--------------------------------------------------------------------------------------------------------*/
#ifndef __BSP_UART_H__
#define __BSP_UART_H__
#include "n32wb452.h"
#include "n32wb452_usart.h"

#include "stdio.h"
#include "ioconfig.h"

// UART标志超时时间
#define UART_FLAG_TIMEOUT ((uint32_t)0x1000)
// UART长超时时间
#define UART_LONG_TIMEOUT ((uint32_t)(10 * UART_FLAG_TIMEOUT))

#define UART_DEVICE_COUNT 3 // 支持的USART数量

// 定义UART接收错误标志位
#define UART_RX_NO_ERROR 0x00    // 无错误
#define UART_RX_OVERRUN_ERR 0x01 // 溢出错误
#define UART_RX_FRAMING_ERR 0x02 // 帧错误
#define UART_RX_PARITY_ERR 0x04  // 奇偶校验错误
#define UART_RX_NOISE_ERR 0x08   // 噪声错误
// 通信模式枚举
typedef enum
{
    BSP_UART_MODE_POLLING, // 轮询模式
    BSP_UART_MODE_IT,      // 中断模式
    BSP_UART_MODE_DMA      // DMA模式
} BSP_UART_CommMode;

// UART设备结构体
typedef struct
{
    USART_Module *uart_base;     // UART外设基地址
    uint32_t baudrate;           // 波特率
    BSP_UART_CommMode comm_mode; // 通信模式

} BSP_UART_Device;

// 状态获取API
uint8_t BSP_UART_GetTxComplete(BSP_UART_Device *device);
uint8_t BSP_UART_GetRxComplete(BSP_UART_Device *device);
uint8_t BSP_UART_GetRxError(BSP_UART_Device *device);

void BSP_UART_Init(BSP_UART_Device *device, USART_Module *uart_base, uint32_t baudrate, BSP_UART_CommMode mode);
int BSP_Uart_Receive(BSP_UART_Device *device, uint8_t *pData, uint16_t size);
int BSP_Uart_Transmit(BSP_UART_Device *device, uint8_t *pData, uint16_t size);
int BSP_Uart_Transmit_IT(BSP_UART_Device *device, uint8_t *pData, uint16_t size);
int BSP_Uart_Receive_IT(BSP_UART_Device *device, uint8_t *pData, uint16_t size);
int BSP_Uart_Transmit_DMA(BSP_UART_Device *device, uint8_t *pData, uint16_t size);
int BSP_Uart_Receive_DMA(BSP_UART_Device *device, uint8_t *pData, uint16_t size);

#endif /* __BSP_UART_H__ */