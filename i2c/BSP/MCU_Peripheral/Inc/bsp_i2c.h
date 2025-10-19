/*-----------------------------------------------File Info------------------------------------------------
** File Name:               bsp_i2c.h  
** Last modified date:      2025.7.1
** Last version:            V0.1
** Description:             I2C驱动接口声明
**
**--------------------------------------------------------------------------------------------------------            
** Created date:            2025.7.1
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            定义I2C编号、DMA模式枚举；
**                          声明I2C初始化、发送/接收函数；
**                          
**--------------------------------------------------------------------------------------------------------*/
#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__

#include "n32wb452.h"
#include "n32wb452_i2c.h"
#include "n32wb452_dma.h"
#include "n32wb452_gpio.h"
#include "stdio.h"
#include "ioconfig.h"

// I2C标志超时时间
#define I2CT_FLAG_TIMEOUT ((uint32_t)0x1000)
// I2C长超时时间
#define I2CT_LONG_TIMEOUT ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))

/**
 * @brief 状态枚举类型
 * 定义操作结果的状态码
 */
typedef enum
{
    FAILED = 0,  // 失败状态
    PASSED = !FAILED  // 成功状态
} Status;

/**
 * @brief 通信控制枚举类型
 * 定义I2C通信过程中的控制状态
 */
typedef enum
{
    C_READY = 0,     // 就绪状态
    C_START_BIT,     // 起始位状态
    C_STOP_BIT       // 停止位状态
}CommCtrl_t;

/**
 * @brief 错误代码枚举类型
 * 定义I2C主从设备可能出现的错误类型
 */
typedef enum
{
    MASTER_OK = 0,      // 主设备正常
    MASTER_BUSY,        // 主设备忙
    MASTER_MODE,        // 主模式错误
    MASTER_TXMODE,      // 主发送模式错误
    MASTER_RXMODE,      // 主接收模式错误
    MASTER_SENDING,     // 主发送中错误
    MASTER_SENDED,      // 主发送完成错误
    MASTER_RECVD,       // 主接收错误
    MASTER_BYTEF,       // 主字节完成错误
    MASTER_BUSERR,      // 主总线错误
    MASTER_UNKNOW,      // 主未知错误
    SLAVE_OK = 20,      // 从设备正常
    SLAVE_BUSY,         // 从设备忙
    SLAVE_MODE,         // 从模式错误
    SLAVE_BUSERR,       // 从总线错误
    SLAVE_UNKNOW,       // 从未知错误

}ErrCode_t;

// 通信模式枚举
typedef enum {
    BSP_I2C_MODE_POLLING,   // 轮询模式
    BSP_I2C_MODE_IT, // 中断模式
    BSP_I2C_MODE_DMA        // DMA模式
} BSP_I2C_CommMode;

// I2C设备结构体
typedef struct {
    I2C_Module* i2c_base;      // I2C外设基地址
    uint8_t device_addr;       // 设备地址
    BSP_I2C_CommMode comm_mode;    // 通信模式
} BSP_I2C_Device;

// 函数声明
void BSP_I2C_Init(BSP_I2C_Device* device, I2C_Module* i2c_base, uint8_t device_addr, BSP_I2C_CommMode mode);
int BSP_I2C_Master_Transmit(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size);
int BSP_I2C_Master_Receive(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size);

// 轮询模式接口
int BSP_I2C_Master_Transmit_Polling(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size);
int BSP_I2C_Master_Receive_Polling(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size);

// 中断模式接口
int BSP_I2C_Master_Transmit_IT(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size);
int BSP_I2C_Master_Receive_IT(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size);
void BSP_I2C_IRQHandler(BSP_I2C_Device* device);

// DMA模式接口
int BSP_I2C_Master_Transmit_DMA(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size);
int BSP_I2C_Master_Receive_DMA(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size);

// 其他功能函数
int BSP_I2C_ScanDevices(BSP_I2C_Device* device, uint8_t* found_addresses, int max_devices);
void CommTimeOut_CallBack(ErrCode_t errcode);
#endif /* __BSP_I2C_H__ */
