/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   主程序主体
  *          .,:,,,                                        .::,,,,::.
  *        .::::,,;;,                                  .,;;:,,.....:i:
  *        :i,.::::,;i:.      ....,,::::::::::,.....   .;i:,.  ......;i.
  *        :;..:::;::::i;,,:::;:,,,,,,,,,,,..,.,,:::iri:. .,:irsr:,.;i.
  *        ;;..,::::;;;;ri,,,.                    ...,,:;s1s1ssrr;,.;r,
  *        :;. ,::;ii;:,     . .....................     .;iirri;;;,,;i,
  *        ,i. .;ri:.   ... ..............................  .,,:;:,,,;i:
  *        :s,.;r:... ......................................... .::;::s;
  *        ,1r::. .............,,,.,,,:,,.........................,;iir;
  *        ,s;...........     ..::.,,;:,,.          ................,;1s
  *       :i,..,.              .,:,,,::,.          .......... ........;1,
  *      ir,....:rrssr;:,       ,,,.,::.     .r5S9989398G95hr;. .....,.:s,
  *     ;r,..,s9855513XHAG3i   .,,,,,,,,.  ,S931,.,,.;s;s&BHHA8s.,...,..:r:
  *    :r;..rGGh,  :SAG;;G@BS:.,,,,,,,,,,.r83:      hHH1sXMBHHHM3..,,,,,.ir.
  *   ,si,.1GS,   sBMAAX&MBMB5,,,,,,,:,,.:&8       3@HXHBMBHBBH#X,.,,.,,,,rr
  *   ;1:,,SH:   .A@&&B#&8H#BS,,,,,,,,,.,5XS,     3@MHABM&59M#As..,,,,,,:,is,
  *  .rr,,,;9&1   hBHHBB&8AMGr,,,,,,,,,,,:h&&9s;   r9&BMHBHMB9:  . ..,,,,;ri.
  *  :1:....:5&XSi;r8BMBHHA9r:,......,,,,:ii19GG88899XHHH&GSr.      ....,:rs.
  *  ;s.     .:sS8G8GG889hi.        ....,,:;:,.:irssrriii:,.        ....,,i1,
  *  ;1,         ..,....,,isssi;,        .,,.                      .....,.i1,
  *  ;h:               i9HHBMBBHAX9:         .                     ....,,,rs,
  *  ,1i..            :A#MBBBBMHB##s                             .....,,,;si.
  *  .r1,..        ,..;3BMBBBHBB#Bh.     ..                    .....,,,,,i1;
  *   :h;..       .,..;,1XBMMMMBXs,.,, .. :: ,.               .....,,,,,,ss.
  *    ih: ..    .;;;, ;;:s58A3i,..    ,. ,.:,,.             ....,,,,,:,s1,
  *    .s1,....   .,;sh,  ,iSAXs;.    ,.  ,,.i85            ....,,,,,,:i1;
  *     .rh: ...     rXG9XBBM#M#MHAX3hss13&&HHXr         ......,,,,,,,ih;
  *      .s5: .....    i598X&&A&AAAAAA&XG851r:       .........,,,,:,,sh;
  *      . ihr, ...  .         ..                    .........,,,,,;11:.
  *         ,s1i. ...  ..,,,..,,,,.,,.,,.,..       .........,,.,,.;s5i.
  *          .:s1r,......................       ...............;shs,
  *          . .:shr:.  ....     .            ..............,ishs.
  *              .,issr;,... ............................,is1s;.
  *                 .,is1si;:,.....................,:;ir1sr;,
  *                    ..:isssssrrii;:::::::;;iirsssssr;:..
  *                         .,::iiirsssssssssrri;;:.
  *
  ******************************************************************************
-----------------------------------------------File Info------------------------------------------------
** File Name:               main.c
** Created date:            2025.7.1
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            C程序入口
**--------------------------------------------------------------------------------------------------------
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

#include "bsp_i2c.h"
#include "bsp_uart.h"
#include "bsp_gpio.h"

// 定义UART设备实例
BSP_UART_Device uart2_device;
BSP_UART_Device uart3_device;
// 接收缓冲区
uint8_t rx_buffer[128];
uint8_t tx_buffer[20] = "hello UART!\r\n";

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
 * @brief 断言失败处理函数
 *
 * 当启用断言检查且断言条件不满足时，会调用此函数
 *
 * @param expr 失败的断言表达式
 * @param file 发生失败的源文件名
 * @param line 发生失败的行号
 */
#ifdef USE_FULL_ASSERT
void assert_failed(const uint8_t *expr, const uint8_t *file, uint32_t line)
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

    BSP_GPIO_Init();
    BSP_GPIO_WritePin(R_LED_GPIO, R_LED_PIN, GPIO_PIN_SET);
    CIRCUIT_SWITCH_UART3_ON();

    // 初始化UART2设备，波特率115200，中断模式
    BSP_UART_Init(&uart2_device, USART2, 115200, BSP_UART_MODE_IT);
    BSP_UART_Init(&uart3_device, USART3, 115200, BSP_UART_MODE_POLLING);

    // 设置全局调试设备指针
    debug_uart_device = &uart3_device;

    while (1)
    {
        // 启动中断模式发送
        BSP_Uart_Transmit_IT(&uart2_device, tx_buffer, sizeof(tx_buffer) - 1);
        BSP_Uart_Transmit_IT(&uart3_device, tx_buffer, sizeof(tx_buffer) - 1);

        BSP_GPIO_TogglePin(G_LED_GPIO, G_LED_PIN);
        BSP_GPIO_TogglePin(R_LED_GPIO, R_LED_PIN);

        /* 插入延时 */
        Delay(0x28FFFF);

        printf("debug test?");
        BSP_GPIO_WritePin(R_LED_GPIO, R_LED_PIN, GPIO_PIN_RESET);
        /* 插入延时 */
        Delay(0x28FFFF);

        /* 点亮Led3 */
        BSP_GPIO_WritePin(G_LED_GPIO, G_LED_PIN, GPIO_PIN_SET);
        /* 插入延时 */
        Delay(0x28FFFF);
        BSP_GPIO_WritePin(G_LED_GPIO, G_LED_PIN, GPIO_PIN_RESET);
        Delay(0x28FFFF);
    }
}
/**
 * @}
 */