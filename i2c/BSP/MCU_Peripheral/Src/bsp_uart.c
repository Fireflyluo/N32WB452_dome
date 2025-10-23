/*-----------------------------------------------File Info------------------------------------------------
** File Name:               bsp_uart.c
** Last modified date:      2025.10.23
** Last version:            V0.1
** Description:             移除全局状态变量，优化为内部静态结构体
**
**--------------------------------------------------------------------------------------------------------
** Created date:            2025.7.1
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            BSP层uart设备初始化代码
**
**--------------------------------------------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"

// 定义全局调试UART设备指针（需在外部初始化）
BSP_UART_Device *debug_uart_device = NULL;

// UART超时计数器
static __IO uint32_t UARTTimeout;

// 中断模式私有状态结构体
typedef struct
{
    uint8_t *tx_buf;              // 发送缓冲区指针
    volatile uint16_t tx_len;     // 发送数据长度
    volatile uint16_t tx_index;   // 发送缓冲区索引
    volatile uint8_t tx_complete; // 发送完成标志

    uint8_t *rx_buf;              // 接收缓冲区指针
    volatile uint16_t rx_len;     // 接收数据长度
    volatile uint16_t rx_index;   // 接收缓冲区索引
    volatile uint8_t rx_complete; // 接收完成标志
    volatile uint8_t rx_error;    // 接收错误标志

    volatile uint16_t rx_received_length; // 实际接收长度
} UART_State;

// 静态全局状态数组
static UART_State uart_states[UART_DEVICE_COUNT];

// 内部函数声明
static void bsp_uart_polingInit(BSP_UART_Device *device);
static void bsp_uart_InterruptInit(BSP_UART_Device *device);
static int bsp_uart_error(BSP_UART_Device *device);

// 通过设备获取状态结构指针
static UART_State *get_uart_state(BSP_UART_Device *device)
{
    if (device->uart_base == USART1)
        return &uart_states[0];
    if (device->uart_base == USART2)
        return &uart_states[1];
    if (device->uart_base == USART3)
        return &uart_states[2];
    return NULL; // 错误处理
}

/**
 * @brief 初始化UART设备
 * @param device UART设备指针
 * @param uart_base UART外设基地址
 * @param baudrate 波特率
 * @param mode 通信模式
 */
void BSP_UART_Init(BSP_UART_Device *device, USART_Module *uart_base, uint32_t baudrate, BSP_UART_CommMode mode)
{
    device->uart_base = uart_base;
    device->baudrate = baudrate;
    device->comm_mode = mode;

    // 初始化私有状态
    UART_State *state = get_uart_state(device);
    if (state)
    {
        state->tx_buf = NULL;
        state->tx_len = 0;
        state->tx_index = 0;
        state->tx_complete = 0;
        state->rx_buf = NULL;
        state->rx_len = 0;
        state->rx_index = 0;
        state->rx_complete = 0;
        state->rx_error = 0;
    }
    if (!state)
    {
        return ; // 错误处理
    }

    // 根据不同模式进行相应初始化
    switch (mode)
    {
    case BSP_UART_MODE_POLLING:
        // 轮询模式初始化
        bsp_uart_polingInit(device);
        break;
    case BSP_UART_MODE_IT:
        // 中断模式初始化，配置NVIC等
        bsp_uart_InterruptInit(device);

        break;
    case BSP_UART_MODE_DMA:
        // DMA模式初始化，使能DMA时钟等
        // bsp_uart_dmaInit(device);
        // 待实现
        break;
    }
}

/**
 * @brief 轮询模式初始化UART外设
 * @param uart_base UART外设基地址
 */
static void bsp_uart_polingInit(BSP_UART_Device *device)
{
    GPIO_InitType GPIO_InitStructure;
    USART_InitType USART_InitStructure;

    if (device->uart_base == USART2)
    {

        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);
        RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USART2, ENABLE);

        USART_DeInit(device->uart_base);

        GPIO_ConfigPinRemap(GPIO_RMP_SW_JTAG_SW_ENABLE, ENABLE); // 禁用JTGA引脚占用
        GPIO_ConfigPinRemap(GPIO_RMP3_USART2, ENABLE);           // USART2引脚复用

        GPIO_InitStruct(&GPIO_InitStructure);
        GPIO_InitStructure.Pin = UART2_RX_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitPeripheral(UART2_RX_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.Pin = UART2_TX_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitPeripheral(UART2_TX_GPIO, &GPIO_InitStructure);

        /*  USART configuration ------------------------------------------------------*/
        USART_StructInit(&USART_InitStructure);
        USART_InitStructure.BaudRate = device->baudrate;
        USART_InitStructure.WordLength = USART_WL_8B;
        USART_InitStructure.StopBits = USART_STPB_1;
        USART_InitStructure.Parity = USART_PE_NO;
        USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
        USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
        /* Configure USART */
        USART_Init(USART2, &USART_InitStructure);

        /* Enable the USART */
        USART_Enable(USART2, ENABLE);
    }
    if (device->uart_base == USART3)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);
        RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USART3, ENABLE);
        USART_DeInit(device->uart_base);
        GPIO_InitStruct(&GPIO_InitStructure);
        GPIO_InitStructure.Pin = UART3_RX_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitPeripheral(UART3_RX_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.Pin = UART3_TX_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitPeripheral(UART3_TX_GPIO, &GPIO_InitStructure);

        /*  USART configuration ------------------------------------------------------*/
        USART_StructInit(&USART_InitStructure);
        USART_InitStructure.BaudRate = device->baudrate;
        USART_InitStructure.WordLength = USART_WL_8B;
        USART_InitStructure.StopBits = USART_STPB_1;
        USART_InitStructure.Parity = USART_PE_NO;
        USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
        USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
        /* Configure USART */
        USART_Init(USART3, &USART_InitStructure);

        /* Enable the USART */
        USART_Enable(USART3, ENABLE);
    }
}
/**
 * @brief 轮询模式发送数据
 * @param device UART设备指针
 * @param data 数据缓冲区
 * @param size 数据大小
 * @return 0成功，其他值失败
 */
int BSP_Uart_Transmit(BSP_UART_Device *device, uint8_t *pData, uint16_t size)
{

    // 参数检查
    if (!device || !pData || size == 0)
    {
        return -1;
    }
    // 逐字节发送数据
    for (int i = 0; i < size; i++)
    {
        UARTTimeout = UART_LONG_TIMEOUT;
        // 等待发送数据寄存器为空
        while (USART_GetFlagStatus(device->uart_base, USART_FLAG_TXDE) == RESET)
        {
            if (UARTTimeout-- == 0)
            {
                return -1; // 超时错误
            }
        }

        // 发送一个字节数据
        USART_SendData(device->uart_base, pData[i]);
    }
    // 等待最后一个字节发送完成
    UARTTimeout = UART_LONG_TIMEOUT;
    while (USART_GetFlagStatus(device->uart_base, USART_FLAG_TXC) == RESET)
    {
        if (UARTTimeout-- == 0)
        {
            return -1; // 超时错误
        }
    }

    return 0;
}

/**
 * @brief 轮询模式接收数据
 * @param device UART设备指针
 * @param pData 数据缓冲区
 * @param size 数据大小
 * @return 0成功，其他值失败
 */
int BSP_Uart_Receive(BSP_UART_Device *device, uint8_t *pData, uint16_t size)
{
    // 参数检查
    if (!device || !pData || size == 0)
    {
        return -1;
    }

    // 逐字节接收数据
    for (int i = 0; i < size; i++)
    {
        UARTTimeout = UART_LONG_TIMEOUT;

        // 等待接收数据寄存器非空
        while (USART_GetFlagStatus(device->uart_base, USART_FLAG_RXDNE) == RESET)
        {
            if (UARTTimeout-- == 0)
            {
                return -1; // 超时错误
            }
        }

        // 读取一个字节数据
        pData[i] = USART_ReceiveData(device->uart_base);
    }

    return 0;
}
/**
 * @brief 中断模式初始化UART外设
 * @param uart_base UART外设基地址
 */
static void bsp_uart_InterruptInit(BSP_UART_Device *device)
{
    GPIO_InitType GPIO_InitStructure;
    USART_InitType USART_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    if (device->uart_base == USART2)
    {
        RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USART2, ENABLE);
        USART_DeInit(device->uart_base);
        RCC_EnableAPB1PeriphClk(RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

        /* Enable the USARTy Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        GPIO_ConfigPinRemap(GPIO_RMP_SW_JTAG_SW_ENABLE, ENABLE); // 禁用JTGA引脚占用
        GPIO_ConfigPinRemap(GPIO_RMP3_USART2, ENABLE);           // USART2引脚复用

        GPIO_InitStruct(&GPIO_InitStructure);
        GPIO_InitStructure.Pin = UART2_RX_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitPeripheral(UART2_RX_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.Pin = UART2_TX_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitPeripheral(UART2_TX_GPIO, &GPIO_InitStructure);

        /*  USART configuration ------------------------------------------------------*/
        USART_StructInit(&USART_InitStructure);
        USART_InitStructure.BaudRate = device->baudrate;
        USART_InitStructure.WordLength = USART_WL_8B;
        USART_InitStructure.StopBits = USART_STPB_1;
        USART_InitStructure.Parity = USART_PE_NO;
        USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
        USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
        /* Configure USART */
        USART_Init(USART2, &USART_InitStructure);
        /* Enable USARTy Receive and Transmit interrupts */
        USART_ConfigInt(USART2, USART_INT_RXDNE, ENABLE);
        USART_ConfigInt(USART2, USART_INT_TXDE, ENABLE);
        /* Enable the USART */
        USART_Enable(USART2, ENABLE);
    }
    if (device->uart_base == USART3)
    {
        RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USART3, ENABLE);
        USART_DeInit(device->uart_base);
        RCC_EnableAPB1PeriphClk(RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

        /* Enable the USARTy Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        GPIO_InitStruct(&GPIO_InitStructure);
        GPIO_InitStructure.Pin = UART3_RX_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitPeripheral(UART3_RX_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.Pin = UART3_TX_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitPeripheral(UART3_TX_GPIO, &GPIO_InitStructure);

        /*  USART configuration ------------------------------------------------------*/
        USART_StructInit(&USART_InitStructure);
        USART_InitStructure.BaudRate = device->baudrate;
        USART_InitStructure.WordLength = USART_WL_8B;
        USART_InitStructure.StopBits = USART_STPB_1;
        USART_InitStructure.Parity = USART_PE_NO;
        USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
        USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
        /* Configure USART */
        USART_Init(USART3, &USART_InitStructure);
        /* Enable USARTy Receive and Transmit interrupts */
        USART_ConfigInt(USART3, USART_INT_RXDNE, ENABLE);
        USART_ConfigInt(USART3, USART_INT_TXDE, ENABLE);
        /* Enable the USART */
        USART_Enable(USART3, ENABLE);
    }
}

/**
 * @brief 中断模式发送数据
 * @param device UART设备指针
 * @param pData 发送数据缓冲区
 * @param size 发送数据长度
 * @retval 0: 成功, -1: 失败
 */
int BSP_Uart_Transmit_IT(BSP_UART_Device *device, uint8_t *pData, uint16_t size)
{
    UART_State *state = get_uart_state(device);
    if (!state || !pData || size == 0)
        return -1;

    // 检查是否忙（发送正在进行）
    if (state->tx_buf != NULL)
        return -1;

    // 设置设备状态
    state->tx_buf = pData;
    state->tx_len = size;
    state->tx_index = 0;
    state->tx_complete = 0;

    USART_SendData(device->uart_base, state->tx_buf[state->tx_index]);
    state->tx_index++;

    USART_ConfigInt(device->uart_base, USART_INT_TXDE, ENABLE);

    return 0;
}

/**
 * @brief  串口接收数据（中断模式）
 * @param  device: 串口设备结构体指针
 * @param  pData: 接收数据缓冲区
 * @param  size: 接收数据长度
 * @retval 0: 成功
 *         -1: 失败
 */
int BSP_Uart_Receive_IT(BSP_UART_Device *device, uint8_t *pData, uint16_t size)
{
    UART_State *state = get_uart_state(device);
    if (!state || !pData || size == 0)
        return -1;

    // 检查是否忙（接收正在进行）
    if (state->rx_buf != NULL)
        return -1;

    // 设置设备状态
    state->rx_buf = pData;
    state->rx_len = size;
    state->rx_index = 0;
    state->rx_complete = 0;
    state->rx_error = 0;

    // 启用RXDNE中断 空闲中断
    USART_ConfigInt(device->uart_base, USART_INT_RXDNE, ENABLE);
    USART_ConfigInt(device->uart_base, USART_INT_IDLEF, ENABLE);
    return 0;
}

// int BSP_Uart_Transmit_DMA(BSP_UART_Device *device, uint8_t *pData, uint16_t size)
//{
// }

/**
 * @brief USART中断服务函数通用处理
 * 处理USART的发送中断请求
 */
static void BSP_UART_IRQHandler(USART_Module *uart_base)
{
    // 创建临时设备结构获取状态
    BSP_UART_Device device = {.uart_base = uart_base};
    UART_State *state = get_uart_state(&device);
    if (!state)
        return;

    // 处理发送数据寄存器空中断(USART_INT_TXDE)
    if (USART_GetIntStatus(device.uart_base, USART_INT_TXDE) != RESET)
    {
        // 清除发送数据寄存器空标志位
        USART_ClrFlag(device.uart_base, USART_FLAG_TXDE);
        // 还有数据
        if (state->tx_index < state->tx_len)
        {
            USART_SendData(device.uart_base, state->tx_buf[state->tx_index]);
            state->tx_index++;
        }
        else
        {
            // 数据发送完成，关闭空中断
            USART_ConfigInt(device.uart_base, USART_INT_TXDE, DISABLE);
            // 使能发送完成中断
            USART_ConfigInt(device.uart_base, USART_INT_TXC, ENABLE);
        }
    }

    // 发送完成中断
    if (USART_GetIntStatus(device.uart_base, USART_INT_TXC) != RESET)
    {
        USART_ClrFlag(device.uart_base, USART_FLAG_TXC);
        USART_ConfigInt(device.uart_base, USART_INT_TXC, DISABLE);

        state->tx_complete = 1;
        state->tx_buf = NULL;
        state->tx_len = 0;
        state->tx_index = 0;
    }

    // 接收中断
    if (USART_GetIntStatus(device.uart_base, USART_INT_RXDNE) != RESET)
    {
        uint8_t received_data = USART_ReceiveData(device.uart_base);
        USART_ClrFlag(device.uart_base, USART_FLAG_RXDNE);

        // 接收数据
        if (state->rx_buf && state->rx_index < state->rx_len)
        {
            state->rx_buf[state->rx_index++] = received_data;

            // 完成接收
            if (state->rx_index >= state->rx_len)
            {
                // 保存实际接收长度
                state->rx_received_length = state->rx_index;
                USART_ConfigInt(device.uart_base, USART_INT_RXDNE, DISABLE);
                state->rx_complete = 1;
            }
        }
    }
    // 空闲中断处理
    if (USART_GetIntStatus(device.uart_base, USART_INT_IDLEF) != RESET)
    {
        // 清除空闲中断标志（必须先读STS再读DAT）
        USART_GetFlagStatus(device.uart_base, USART_FLAG_IDLEF);
        USART_ReceiveData(device.uart_base);
        // 未达到设定接收长度提前接收完成
        if (state->rx_buf && state->rx_index < state->rx_len)
        {
            // 保存实际接收长度
            state->rx_received_length = state->rx_index;
            // 接收完成
            state->rx_complete = 1;
            USART_ConfigInt(device.uart_base, USART_INT_RXDNE, DISABLE);
            USART_ConfigInt(device.uart_base, USART_INT_IDLEF, DISABLE);
        }
    }

    // 错误中断
    int error_status = bsp_uart_error(&device);
}

/**
 * @brief USART2中断服务函数
 * 处理USART2的发送中断请求
 */
void USART2_IRQHandler(void)
{
    BSP_UART_IRQHandler(USART2);
}

/**
 * @brief USART3中断服务函数
 * 处理USART3的发送中断请求
 */
void USART3_IRQHandler(void)
{
    BSP_UART_IRQHandler(USART3);
}
/**
 * @brief  UART错误处理函数
 * @param  device: UART设备指针
 * @retval 无
 */
int bsp_uart_error(BSP_UART_Device *device)
{
    UART_State *state = get_uart_state(device);
    if (!state)
        return -1;

    uint32_t status = 0;

    // 检查各种错误标志
    if (USART_GetFlagStatus(device->uart_base, USART_INT_OREF) != RESET)
    {
        status = device->uart_base->STS;
        (void)device->uart_base->DAT; // 清除错误
        state->rx_error |= UART_RX_OVERRUN_ERR;
    }
    if (USART_GetFlagStatus(device->uart_base, USART_INT_FEF) != RESET)
    {
        status = device->uart_base->STS;
        (void)device->uart_base->DAT;
        state->rx_error |= UART_RX_FRAMING_ERR;
    }
    if (USART_GetFlagStatus(device->uart_base, USART_INT_PEF) != RESET)
    {
        status = device->uart_base->STS;
        (void)device->uart_base->DAT;
        state->rx_error |= UART_RX_PARITY_ERR;
    }
    if (USART_GetFlagStatus(device->uart_base, USART_INT_NEF) != RESET)
    {
        status = device->uart_base->STS;
        (void)device->uart_base->DAT;
        state->rx_error |= UART_RX_NOISE_ERR;
    }

    return status;
}
/**
 * @brief 状态重置
 * @param device UART设备指针
 * @return 0: 成功, -1: 失败
 */
int BSP_Uart_Receive_Finish(BSP_UART_Device *device)
{
    UART_State *state = get_uart_state(device);
    if (!state || !state->rx_complete)
        return -1; // 还未完成接收

    // 重置接收状态
    state->rx_buf = NULL;
    state->rx_len = 0;
    state->rx_index = 0;
    state->rx_complete = 0;
    state->rx_error = 0;

    return 0;
}
/**
 * @brief 获取发送完成状态
 * @param device UART设备指针
 * @return 1: 完成, 0: 未完成
 */
uint8_t BSP_UART_GetTxComplete(BSP_UART_Device *device)
{
    UART_State *state = get_uart_state(device);
    return (state) ? state->tx_complete : 0;
}

/**
 * @brief 获取接收完成状态
 * @param device UART设备指针
 * @return 1: 完成, 0: 未完成
 */
uint8_t BSP_UART_GetRxComplete(BSP_UART_Device *device)
{
    UART_State *state = get_uart_state(device);
    return (state) ? state->rx_complete : 0;
}

/**
 * @brief 获取接收错误状态
 * @param device UART设备指针
 * @return 错误标志位组合
 */
uint8_t BSP_UART_GetRxError(BSP_UART_Device *device)
{
    UART_State *state = get_uart_state(device);
    return (state) ? state->rx_error : 0;
}
/**
 * @brief 获取实际接收的数据长度
 * @param device UART设备指针
 * @return 实际接收的字节数
 */
uint16_t BSP_UART_GetRxLength(BSP_UART_Device *device)
{
    UART_State *state = get_uart_state(device);
    return (state) ? state->rx_received_length : 0;
}

/******************************************串口重定向*****************************************************************/
int fputc(int ch, FILE *f)
{
    if (debug_uart_device)
    {
        // 使用轮询模式发送单个字符
        BSP_Uart_Transmit(debug_uart_device, (uint8_t *)&ch, 1);
    }
    return ch;
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    if (debug_uart_device)
    {
        // 使用轮询模式发送单个字符
        BSP_Uart_Transmit(debug_uart_device, (uint8_t *)&ch, 1);
    }
    return ch;
}

#ifdef __GNUC__
int _write(int fd, char *ptr, int len)
{
    if (debug_uart_device)
    {
        // 使用轮询模式发送整个字符串
        BSP_Uart_Transmit(debug_uart_device, (uint8_t *)ptr, len);
    }
    return len;
}
#endif