#include "bsp_i2c.h"


// 非可重入标志，用于防止函数并发执行
#ifdef NON_REENTRANT
static uint32_t Mutex_Flag = 0;
#endif

// 中断模式相关全局变量
static BSP_I2C_Device* i2c_current_device = NULL;
static uint8_t* i2c_tx_buffer = NULL;
static uint8_t* i2c_rx_buffer = NULL;
static uint16_t i2c_tx_size = 0;
static uint16_t i2c_rx_size = 0;
static uint8_t i2c_reg_addr = 0xFF;
static uint8_t i2c_direction = 0; // 0: write, 1: read

// I2C超时计数器
static __IO uint32_t I2CTimeout;
// 通信控制标志
static CommCtrl_t Comm_Flag = C_READY;

// 内部函数声明
static void bsp_i2c_polingInit(I2C_Module* i2c_base);

/**
 * @brief 初始化I2C设备
 * @param device I2C设备指针
 * @param i2c_base I2C外设基地址
 * @param device_addr 设备地址
 * @param mode 通信模式
 */
void BSP_I2C_Init(BSP_I2C_Device* device, I2C_Module* i2c_base, uint8_t device_addr, BSP_I2C_CommMode mode)
{
    device->i2c_base = i2c_base;
    device->device_addr = device_addr;
    device->comm_mode = mode;
    
    // 根据不同模式进行相应初始化
    switch(mode) {
        case BSP_I2C_MODE_POLLING:
            // 轮询模式初始化
            bsp_i2c_polingInit(i2c_base);
            break;
        case BSP_I2C_MODE_INTERRUPT:
            // 中断模式初始化，配置NVIC等


            break;
        case BSP_I2C_MODE_DMA:
            // DMA模式初始化，使能DMA时钟等
            RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA1, ENABLE);
            break;
    }
}
/**
 * @brief 通用I2C发送数据接口
 * @param device I2C设备指针
 * @param reg_addr 寄存器地址
 * @param data 数据缓冲区
 * @param size 数据大小
 * @return 0成功，其他值失败
 */
int BSP_I2C_Master_Transmit(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size)
{
    switch(device->comm_mode) {
        case BSP_I2C_MODE_POLLING:
            return BSP_I2C_Master_Transmit_Polling(device, reg_addr, data, size);
        case BSP_I2C_MODE_INTERRUPT:
            return BSP_I2C_Master_Transmit_Interrupt(device, reg_addr, data, size);
        case BSP_I2C_MODE_DMA:
            return BSP_I2C_Master_Transmit_DMA(device, reg_addr, data, size);
        default:
            return -1;
    }
}
/**
 * @brief 通用I2C接收数据接口
 * @param device I2C设备指针
 * @param reg_addr 寄存器地址
 * @param data 数据缓冲区
 * @param size 数据大小
 * @return 0成功，其他值失败
 */
int BSP_I2C_Master_Receive(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size)
{
    switch(device->comm_mode) {
        case BSP_I2C_MODE_POLLING:
            return BSP_I2C_Master_Receive_Polling(device, reg_addr, data, size);
        case BSP_I2C_MODE_INTERRUPT:
            return BSP_I2C_Master_Receive_Interrupt(device, reg_addr, data, size);
        case BSP_I2C_MODE_DMA:
            return BSP_I2C_Master_Receive_DMA(device, reg_addr, data, size);
        default:
            return -1;
    }
}


// 轮询模式初始化函数
static void bsp_i2c_polingInit(I2C_Module* i2c_base)
{
    GPIO_InitType GPIO_InitStructure;
    I2C_InitType I2C_InitStructure;
    
    // 根据 i2c_base 确定使用的时钟和引脚
    if (i2c_base == I2C1) {
        RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_I2C1, ENABLE);
        I2C_DeInit(i2c_base);
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);

        GPIOB->POD |= (I2C1_SCL_PIN | I2C1_SDA_PIN); //pull up PB8\PB9
        
        GPIO_InitStruct(&GPIO_InitStructure);
        GPIO_InitStructure.Pin        = I2C1_SCL_PIN | I2C1_SDA_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
        GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

        /** I2C peripheral configuration */
        I2C_DeInit(i2c_base);
        I2C_InitStruct(&I2C_InitStructure);
        I2C_InitStructure.BusMode     = I2C_BUSMODE_I2C;
        I2C_InitStructure.FmDutyCycle = I2C_FMDUTYCYCLE_2;
        I2C_InitStructure.OwnAddr1    = 0xff;
        I2C_InitStructure.AckEnable   = I2C_ACKEN;
        I2C_InitStructure.AddrMode    = I2C_ADDR_MODE_7BIT;
        I2C_InitStructure.ClkSpeed    = 100000;
        I2C_Init(i2c_base, &I2C_InitStructure);


    } else if (i2c_base == I2C2) {
        RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_I2C2, ENABLE);
        I2C_DeInit(i2c_base);
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);

        GPIOB->POD |= (I2C2_SCL_PIN | I2C2_SDA_PIN); //pull up PB10\PB11

        GPIO_InitStruct(&GPIO_InitStructure);
        GPIO_InitStructure.Pin        = I2C2_SCL_PIN | I2C2_SDA_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
        GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

        /** I2C peripheral configuration */
        I2C_DeInit(i2c_base);
        I2C_InitStruct(&I2C_InitStructure);
        I2C_InitStructure.BusMode     = I2C_BUSMODE_I2C;
        I2C_InitStructure.FmDutyCycle = I2C_FMDUTYCYCLE_2;
        I2C_InitStructure.OwnAddr1    = 0xff;
        I2C_InitStructure.AckEnable   = I2C_ACKEN;
        I2C_InitStructure.AddrMode    = I2C_ADDR_MODE_7BIT;
        I2C_InitStructure.ClkSpeed    = 100000;
        I2C_Init(i2c_base, &I2C_InitStructure);

    }
    
    
    
}




/**
 * @brief 轮询模式发送数据
 * @param device I2C设备指针
 * @param reg_addr 外设寄存器地址（如果不需要可设为0xFF）
 * @param data 数据缓冲区
 * @param size 数据大小
 * @return 0成功，其他值失败
 */
int BSP_I2C_Master_Transmit_Polling(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size)
{
    uint8_t* sendBufferPtr = data;

#ifdef NON_REENTRANT
    // 检查是否可重入，防止并发执行
    if (Mutex_Flag)
        return -1;
    else
        Mutex_Flag = 1;
#endif

    I2CTimeout = I2CT_LONG_TIMEOUT;
    // 等待总线空闲
    while (I2C_GetFlag(device->i2c_base, I2C_FLAG_BUSY))
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_BUSY);  // 总线忙
             goto error_cleanup;
        }
    }

    // 发送起始条件
    if (Comm_Flag == C_READY)
    {
        Comm_Flag = C_START_BIT;
        I2C_GenerateStart(device->i2c_base, ENABLE);
    }

    I2CTimeout = I2CT_LONG_TIMEOUT;
    // 等待主模式标志(EV5事件)
    while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_MODE_FLAG))
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_MODE);// 主模式错误
           goto error_cleanup;
        }
    }
     
    // 发送从设备地址(写模式)
    I2C_SendAddr7bit(device->i2c_base, device->device_addr, I2C_DIRECTION_SEND);  
    
    I2CTimeout = I2CT_LONG_TIMEOUT;
    // 等待发送模式标志(EV6事件)
    while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_TXMODE_FLAG))
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_TXMODE);
             goto error_cleanup;
        }
    }
    Comm_Flag = C_READY;

    
    // 如果需要发送寄存器地址
    if (reg_addr != 0xFF) {
        I2C_SendData(device->i2c_base, reg_addr);
        
        I2CTimeout = I2CT_LONG_TIMEOUT;
        while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_DATA_SENDED)) 
        {
            if ((I2CTimeout--) == 0)
            {
                CommTimeOut_CallBack(MASTER_TXMODE);
                 goto error_cleanup;
            }
        
        }
    }
    
    // 发送数据
    while (size--) {
        I2C_SendData(device->i2c_base, *data++);
        
        I2CTimeout = I2CT_LONG_TIMEOUT;
        // 等待数据发送中标志(EV8事件)
        while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_DATA_SENDING))
        {
            if ((I2CTimeout--) == 0)
            {
                CommTimeOut_CallBack(MASTER_SENDING);
                 goto error_cleanup;
            }
        }
    }

    I2CTimeout = I2CT_LONG_TIMEOUT;
    // 等待数据发送完成标志(EV8-2事件)
    while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_DATA_SENDED))
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_SENDED);
             goto error_cleanup;
        }
    }
    // 发送停止条件
    if (Comm_Flag == C_READY)
    {
        Comm_Flag = C_STOP_BIT;
        I2C_GenerateStop(device->i2c_base, ENABLE);
    }
   
    // 等待总线空闲
    while (I2C_GetFlag(device->i2c_base, I2C_FLAG_BUSY))
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_BUSY);
             goto error_cleanup;
        }
    }
    Comm_Flag = C_READY;    // 就绪

    #ifdef NON_REENTRANT // 非重入保护
    // 释放互斥锁
    if (Mutex_Flag)
        Mutex_Flag = 0;
    else
        return -2;
#endif
    
    return 0;



error_cleanup:
    I2C_GenerateStop(device->i2c_base, ENABLE);
    Comm_Flag = C_READY;
#ifdef NON_REENTRANT
    Mutex_Flag = 0;
#endif
    return 0;
}

/**
 * @brief 轮询模式接收数据
 * @param device I2C设备指针
 * @param reg_addr 寄存器地址（如果不需要可设为0xFF）
 * @param data 数据缓冲区
 * @param size 数据大小
 * @return 0成功，其他值失败
 */
int BSP_I2C_Master_Receive_Polling(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size)
{
    uint8_t* recvBufferPtr = data;
    
#ifdef NON_REENTRANT
    // 检查是否可重入，防止并发执行
    if (Mutex_Flag)
        return -1;
    else
        Mutex_Flag = 1;
#endif
/*--- 阶段1: 总线初始化 ---*/
  I2CTimeout = I2CT_LONG_TIMEOUT;
    // 等待I2C总线空闲
    while (I2C_GetFlag(device->i2c_base, I2C_FLAG_BUSY))
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_BUSY);  // 总线忙
            goto error_cleanup;
        }
    }
    
    // 清除NACK位置位
    (device->i2c_base)->CTRL1 &= ~I2C_NACK_POS_NEXT;
    I2C_ConfigAck(device->i2c_base, ENABLE);
    
/*--- 阶段2: 发送起始条件 ---*/
    if (Comm_Flag == C_READY)
    {
        Comm_Flag = C_START_BIT;
        I2C_GenerateStart(device->i2c_base, ENABLE);
    }

    I2CTimeout = I2CT_LONG_TIMEOUT;
    // 等待主模式标志(EV5事件)
    while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_MODE_FLAG))
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_MODE);
            goto error_cleanup;
        }
    }
    
/*--- 阶段3: 发送设备地址(写模式) ---*/
    I2C_SendAddr7bit(device->i2c_base, device->device_addr, I2C_DIRECTION_SEND); // EV6

    I2CTimeout = I2CT_LONG_TIMEOUT;
    // 等待发送模式标志(EV6事件)
    while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_TXMODE_FLAG))
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_TXMODE);
             goto error_cleanup;
        }
    }
    
    Comm_Flag = C_READY;

/*--- 阶段4: 寄存器地址处理 ---*/
    if (reg_addr != 0xFF) {
        // 通过重新设置PE位清除EV6
        I2C_Enable(device->i2c_base, ENABLE);

        // 发送内部要读取的寄存器地址
        I2C_SendData(device->i2c_base, reg_addr);

        // 检查EV8事件并清除
        I2CTimeout = I2CT_LONG_TIMEOUT;
        while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_DATA_SENDED))
        {
            if ((I2CTimeout--) == 0)
            {
                CommTimeOut_CallBack(MASTER_TXMODE);
                 goto error_cleanup;
            }
        }
        
        // 第二次发送起始条件（重复起始）EV5
        I2C_GenerateStart(device->i2c_base, ENABLE);

        // 检查EV5事件并清除
        I2CTimeout = I2CT_LONG_TIMEOUT;
        while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_MODE_FLAG))
        {
            if ((I2CTimeout--) == 0)
            {
                CommTimeOut_CallBack(MASTER_MODE);
                 goto error_cleanup;
            }
        }
        
        // 发送设备地址（读模式）EV6
        I2C_SendAddr7bit(device->i2c_base, device->device_addr, I2C_DIRECTION_RECV);

        I2CTimeout = I2CT_LONG_TIMEOUT;
        while (!I2C_GetFlag(device->i2c_base, I2C_EVT_MASTER_RXMODE_FLAG))
        {
            if ((I2CTimeout--) == 0)
            {
                CommTimeOut_CallBack(MASTER_MODE);
                goto error_cleanup;
            }
        }
    }
/*--- 阶段5: 清除ADDR标志 ---*/
    (void)(device->i2c_base)->STS1; // RM 15.4.1 要求双寄存器读取
    (void)(device->i2c_base)->STS2;

/*--- 阶段6: 数据接收处理 ---*/
    if (size == 1)
    {
        // 接收1个字节
        I2C_ConfigAck(device->i2c_base, DISABLE); // 禁用应答
        if (Comm_Flag == C_READY)
        {
            Comm_Flag = C_STOP_BIT;
            I2C_GenerateStop(device->i2c_base, ENABLE);
        }
        
        I2CTimeout = I2CT_LONG_TIMEOUT;
        // 等待接收数据非空中断标志
        while (!I2C_GetFlag(device->i2c_base, I2C_EVT_MASTER_DATA_RECVD_FLAG))
        {
            if ((I2CTimeout--) == 0)
            {
                CommTimeOut_CallBack(MASTER_RECVD);
                 goto error_cleanup;
            }
        }
        *recvBufferPtr++ = I2C_RecvData(device->i2c_base);
    }
    else if (size == 2)
    {
        // 双字节优化模式 (RM 15.4.8)
        I2C_ConfigAck(device->i2c_base, ENABLE); // 第一个字节ACK
        // 接收第一个字节
        I2CTimeout = I2CT_LONG_TIMEOUT;
        while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_DATA_RECVD_FLAG)) {
            if (I2CTimeout-- == 0) {
                CommTimeOut_CallBack(MASTER_RECVD);
                 goto error_cleanup;
            }
        }
        *recvBufferPtr++ = I2C_RecvData(device->i2c_base);
        
        // 准备接收最后一个字节
        I2C_ConfigAck(device->i2c_base, DISABLE); // 最后一个字节NACK
        I2C_GenerateStop(device->i2c_base, ENABLE);

        // 接收第二个字节
        I2CTimeout = I2CT_LONG_TIMEOUT;
        while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_DATA_RECVD_FLAG)) {
            if (I2CTimeout-- == 0) {
                CommTimeOut_CallBack(MASTER_RECVD);
                 goto error_cleanup;
            }
        }
        *recvBufferPtr = I2C_RecvData(device->i2c_base);
    } 
    else {
        // 多字节接收 (size >= 3)
        uint16_t bytes_remaining = size;
        
        while (bytes_remaining > 0) {
            if (bytes_remaining == 1) {
                // 最后一个字节: NACK + STOP
                I2C_ConfigAck(device->i2c_base, DISABLE);
                I2C_GenerateStop(device->i2c_base, ENABLE);
            } 
            else if (bytes_remaining == 2) {
                // 倒数第二个字节: 保持ACK
                I2C_ConfigAck(device->i2c_base, ENABLE);
            }
            
            // 等待数据接收完成
            I2CTimeout = I2CT_LONG_TIMEOUT;
            while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_DATA_RECVD_FLAG)) {
                if (I2CTimeout-- == 0) {
                    CommTimeOut_CallBack(MASTER_RECVD);
                     goto error_cleanup;
                }
            }
            
            // 读取数据并更新指针
            *recvBufferPtr++ = I2C_RecvData(device->i2c_base);
            bytes_remaining--;
        }
    }

     /*--- 阶段7: 最终总线检查 ---*/
    I2CTimeout = I2CT_LONG_TIMEOUT;
    while (I2C_GetFlag(device->i2c_base, I2C_FLAG_BUSY)) {
        if (I2CTimeout-- == 0) {
            CommTimeOut_CallBack(MASTER_BUSY);
             goto error_cleanup;
        }
    }
    
    Comm_Flag = C_READY;
    
#ifdef NON_REENTRANT
    // 释放互斥锁
    if (Mutex_Flag)
        Mutex_Flag = 0;
    else
        return -2;
#endif
    
    return 0;


error_cleanup:
    I2C_GenerateStop(device->i2c_base, ENABLE);
    Comm_Flag = C_READY;
#ifdef NON_REENTRANT
    Mutex_Flag = 0;
#endif
    return 0;
}

/**
 * @brief 中断模式发送数据
 * @param device I2C设备指针
 * @param reg_addr 寄存器地址
 * @param data 数据缓冲区
 * @param size 数据大小
 * @return 0成功，其他值失败
 */
int BSP_I2C_Master_Transmit_Interrupt(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size)
{
    // 初始化全局变量
    i2c_current_device = device;
    i2c_tx_buffer = data;
    i2c_tx_size = size;
    i2c_reg_addr = reg_addr;
    i2c_direction = 0; // write
    
    // 配置中断
    I2C_ConfigInt(device->i2c_base, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, ENABLE);
    
    // 产生起始条件
    I2C_GenerateStart(device->i2c_base, ENABLE);
    
    return 0;
}

/**
 * @brief 中断模式接收数据
 * @param device I2C设备指针
 * @param reg_addr 寄存器地址
 * @param data 数据缓冲区
 * @param size 数据大小
 * @return 0成功，其他值失败
 */
int BSP_I2C_Master_Receive_Interrupt(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size)
{
    // 初始化全局变量
    i2c_current_device = device;
    i2c_rx_buffer = data;
    i2c_rx_size = size;
    i2c_reg_addr = reg_addr;
    i2c_direction = 1; // read
    
    // 配置中断
    I2C_ConfigInt(device->i2c_base, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, ENABLE);
    
    // 产生起始条件
    I2C_GenerateStart(device->i2c_base, ENABLE);
    
    return 0;
}

/**
 * @brief I2C事件中断处理函数
 * @param device I2C设备指针
 */
void BSP_I2C_IRQHandler(BSP_I2C_Device* device)
{
    uint32_t last_event = I2C_GetLastEvent(device->i2c_base);
    
    switch (last_event) {
        case I2C_EVT_MASTER_MODE_FLAG:
            if (i2c_direction == 0) { // write
                I2C_SendAddr7bit(device->i2c_base, device->device_addr, I2C_DIRECTION_SEND);
            } else { // read
                if (i2c_reg_addr != 0xFF) {
                    I2C_SendAddr7bit(device->i2c_base, device->device_addr, I2C_DIRECTION_SEND);
                } else {
                    I2C_SendAddr7bit(device->i2c_base, device->device_addr, I2C_DIRECTION_RECV);
                }
            }
            break;
            
        case I2C_EVT_MASTER_TXMODE_FLAG:
            if (i2c_reg_addr != 0xFF) {
                I2C_SendData(device->i2c_base, i2c_reg_addr);
                i2c_reg_addr = 0xFF; // 清除寄存器地址
            } else if (i2c_tx_size > 0) {
                I2C_SendData(device->i2c_base, *i2c_tx_buffer++);
                i2c_tx_size--;
            }
            break;
            
        case I2C_EVT_MASTER_DATA_SENDED:
            if (i2c_tx_size > 0) {
                I2C_SendData(device->i2c_base, *i2c_tx_buffer++);
                i2c_tx_size--;
            } else if (i2c_direction == 1 && i2c_reg_addr == 0xFF) {
                // 读操作，重新开始
                I2C_GenerateStart(device->i2c_base, ENABLE);
            } else {
                I2C_GenerateStop(device->i2c_base, ENABLE);
                // 可以设置完成标志或回调
            }
            break;
            
        case I2C_EVT_MASTER_RXMODE_FLAG:
            // 配置ACK策略
            if (i2c_rx_size == 1) {
                I2C_ConfigAck(device->i2c_base, DISABLE);
                I2C_GenerateStop(device->i2c_base, ENABLE);
            } else if (i2c_rx_size == 2) {
                device->i2c_base->CTRL1 |= I2C_NACK_POS_NEXT;
                I2C_ConfigAck(device->i2c_base, DISABLE);
            } else {
                I2C_ConfigAck(device->i2c_base, ENABLE);
            }
            (void)(device->i2c_base->STS1);
            (void)(device->i2c_base->STS2);
            break;
            
        case I2C_EVT_MASTER_DATA_RECVD_FLAG:
            *i2c_rx_buffer++ = I2C_RecvData(device->i2c_base);
            i2c_rx_size--;
            
            if (i2c_rx_size == 1) {
                I2C_ConfigAck(device->i2c_base, DISABLE);
                I2C_GenerateStop(device->i2c_base, ENABLE);
            } else if (i2c_rx_size == 0) {
                // 传输完成，可以设置标志或回调
            }
            break;
    }
}

/**
 * @brief DMA模式发送数据
 * @param device I2C设备指针
 * @param reg_addr 寄存器地址
 * @param data 数据缓冲区
 * @param size 数据大小
 * @return 0成功，其他值失败
 */
int BSP_I2C_Master_Transmit_DMA(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size)
{
    DMA_InitType DMA_InitStructure;
    uint32_t timeout = 0xFFFFF;
    
    // 等待总线空闲
    while (I2C_GetFlag(device->i2c_base, I2C_FLAG_BUSY)) {
        if ((timeout--) == 0) return -1;
    }
    
    // 配置DMA通道
    DMA_DeInit(DMA1_CH6);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.PeriphAddr = (uint32_t)&(device->i2c_base->DAT);
    DMA_InitStructure.MemAddr = (uint32_t)data;
    DMA_InitStructure.Direction = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize = size;
    DMA_InitStructure.PeriphInc = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStructure.MemDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.CircularMode = DMA_MODE_NORMAL;
    DMA_InitStructure.Priority = DMA_PRIORITY_VERY_HIGH;
    DMA_InitStructure.Mem2Mem = DMA_M2M_DISABLE;
    DMA_Init(DMA1_CH6, &DMA_InitStructure);
    
    // 产生起始条件
    I2C_GenerateStart(device->i2c_base, ENABLE);
    
    // 等待主模式标志
    timeout = 0xFFFFF;
    while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_MODE_FLAG)) {
        if ((timeout--) == 0) return -1;
    }
    
    // 发送设备地址（写模式）
    I2C_SendAddr7bit(device->i2c_base, device->device_addr, I2C_DIRECTION_SEND);
    
    // 等待发送模式标志
    timeout = 0xFFFFF;
    while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_TXMODE_FLAG)) {
        if ((timeout--) == 0) return -1;
    }
    
    // 发送寄存器地址
    if (reg_addr != 0xFF) {
        I2C_SendData(device->i2c_base, reg_addr);
        
        timeout = 0xFFFFF;
        while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_DATA_SENDING)) {
            if ((timeout--) == 0) return -1;
        }
    }
    
    // 使能DMA
    DMA_EnableChannel(DMA1_CH6, ENABLE);
    I2C_EnableDMA(device->i2c_base, ENABLE);
    
    // 等待传输完成
    timeout = 0xFFFFF;
    while (!I2C_GetFlag(device->i2c_base, I2C_FLAG_BYTEF)) {
        if ((timeout--) == 0) return -1;
    }
    
    // 产生停止条件
    I2C_GenerateStop(device->i2c_base, ENABLE);
    
    // 禁用DMA
    I2C_EnableDMA(device->i2c_base, DISABLE);
    DMA_EnableChannel(DMA1_CH6, DISABLE);
    
    return 0;
}

/**
 * @brief DMA模式接收数据
 * @param device I2C设备指针
 * @param reg_addr 寄存器地址
 * @param data 数据缓冲区
 * @param size 数据大小
 * @return 0成功，其他值失败
 */
int BSP_I2C_Master_Receive_DMA(BSP_I2C_Device* device, uint8_t reg_addr, uint8_t* data, uint16_t size)
{
    DMA_InitType DMA_InitStructure;
    uint32_t timeout = 0xFFFFF;
    
    // 先写入寄存器地址
    if (reg_addr != 0xFF) {
        if (BSP_I2C_Master_Transmit_Polling(device, reg_addr, NULL, 0) != 0) {
            return -1;
        }
    }
    
    // 等待总线空闲
    while (I2C_GetFlag(device->i2c_base, I2C_FLAG_BUSY)) {
        if ((timeout--) == 0) return -1;
    }
    
    // 配置DMA通道
    if (size > 1) {
        DMA_DeInit(DMA1_CH7);
        DMA_StructInit(&DMA_InitStructure);
        DMA_InitStructure.PeriphAddr = (uint32_t)&(device->i2c_base->DAT);
        DMA_InitStructure.MemAddr = (uint32_t)data;
        DMA_InitStructure.Direction = DMA_DIR_PERIPH_SRC;
        DMA_InitStructure.BufSize = size;
        DMA_InitStructure.PeriphInc = DMA_PERIPH_INC_DISABLE;
        DMA_InitStructure.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
        DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
        DMA_InitStructure.MemDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.CircularMode = DMA_MODE_NORMAL;
        DMA_InitStructure.Priority = DMA_PRIORITY_VERY_HIGH;
        DMA_InitStructure.Mem2Mem = DMA_M2M_DISABLE;
        DMA_Init(DMA1_CH7, &DMA_InitStructure);
    }
    
    // 产生起始条件
    I2C_GenerateStart(device->i2c_base, ENABLE);
    
    // 等待主模式标志
    timeout = 0xFFFFF;
    while (!I2C_CheckEvent(device->i2c_base, I2C_EVT_MASTER_MODE_FLAG)) {
        if ((timeout--) == 0) return -1;
    }
    
    // 发送设备地址（读模式）
    I2C_SendAddr7bit(device->i2c_base, device->device_addr, I2C_DIRECTION_RECV);
    
    // 等待地址标志
    timeout = 0xFFFFF;
    while (!I2C_GetFlag(device->i2c_base, I2C_FLAG_ADDRF)) {
        if ((timeout--) == 0) return -1;
    }
    
    if (size == 1) {
        I2C_ConfigAck(device->i2c_base, DISABLE);
        (void)(device->i2c_base->STS1);
        (void)(device->i2c_base->STS2);
        I2C_GenerateStop(device->i2c_base, ENABLE);
        
        timeout = 0xFFFFF;
        while (!I2C_GetFlag(device->i2c_base, I2C_FLAG_RXDATNE)) {
            if ((timeout--) == 0) return -1;
        }
        *data = I2C_RecvData(device->i2c_base);
    } else {
        I2C_ConfigAck(device->i2c_base, ENABLE);
        (void)(device->i2c_base->STS1);
        (void)(device->i2c_base->STS2);
        I2C_EnableDmaLastSend(device->i2c_base, ENABLE);
        
        DMA_EnableChannel(DMA1_CH7, ENABLE);
        I2C_EnableDMA(device->i2c_base, ENABLE);
        
        timeout = 0xFFFFF;
        while (!(DMA_GetFlagStatus(DMA1_FLAG_TC7, DMA1))) {
            if ((timeout--) == 0) return -1;
        }
        
        I2C_EnableDMA(device->i2c_base, DISABLE);
        DMA_EnableChannel(DMA1_CH7, DISABLE);
        I2C_EnableDmaLastSend(device->i2c_base, DISABLE);
        I2C_GenerateStop(device->i2c_base, ENABLE);
    }
    
    return 0;
}

/**
 * @brief 扫描I2C总线上的设备
 * @param device I2C设备指针
 * @param found_addresses 存储找到的设备地址的数组
 * @param max_devices 数组最大容量
 * @return 找到的设备数量
 */
/**
 * @brief 扫描I2C总线上的设备（基于轮询模式读写函数的实现）
 * @param device I2C设备指针
 * @param found_addresses 存储找到的设备地址的数组
 * @param max_devices 数组最大容量
 * @return 找到的设备数量
 */
int BSP_I2C_ScanDevices(BSP_I2C_Device* device, uint8_t* found_addresses, int max_devices)
{
    int found_count = 0;
    int result;
    
    // 遍历所有可能的7位I2C地址 (0x08到0x77是合法的7位地址范围)
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        // 检查数组是否已满
        if (found_count >= max_devices) {
            break;
        }
        
        // 临时保存原始设备地址
        uint8_t original_addr = device->device_addr;
        
        // 设置当前扫描的设备地址
        device->device_addr = addr;
        
        // 尝试向设备发送一个字节的数据（寄存器地址设为0xFF表示不发送寄存器地址）
        // 这里使用一个简单的写操作来检测设备是否存在
        result = BSP_I2C_Master_Transmit_Polling(device, 0xFF, (uint8_t*)"", 0);
        
        // 恢复原始设备地址
        device->device_addr = original_addr;
        
        // 如果发送成功，说明设备存在
        if (result == 0) {
            found_addresses[found_count++] = addr;
        }
        
        // 添加小延迟以避免总线过于繁忙
        for(volatile int i = 0; i < 1000; i++);
    }
    
    return found_count;
}
/**
 * @brief  通信超时回调函数
 * @param errcode 错误代码
 * 
 * 处理I2C通信超时错误，根据配置的恢复模式进行相应处理
 */
void CommTimeOut_CallBack(ErrCode_t errcode)
{
    Comm_Flag = C_READY;
    
#if (COMM_RECOVER_MODE == MODULE_SELF_RESET)
//    IIC_SWReset();
#elif (COMM_RECOVER_MODE == MODULE_RCC_RESET)
    IIC_RCCReset();
#elif (COMM_RECOVER_MODE == SYSTEM_NVIC_RESET)
    SystemNVICReset();
#endif
}
