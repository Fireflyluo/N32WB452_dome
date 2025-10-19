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
#include "bsp_gpio.h"
     
BSP_I2C_Device i2c_device;
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
 * @brief I2C 轮询模式测试函数
 * 
 * 此函数演示如何使用轮询模式进行 I2C 通信
 */

void I2C_Polling_Test(void)
{
    uint8_t test_data[4] = {0x12, 0x34, 0x56, 0x78};
    uint8_t read_data[4] = {0};
    uint8_t found_devices[120]={0};  // 存储找到的设备地址
    int result;

    // 初始化 I2C 设备为轮询模式
    BSP_I2C_Init(&i2c_device, I2C1, 0xD2, BSP_I2C_MODE_POLLING); // 设备地址为 0x69 << 1
//    BSP_I2C_Master_Transmit(&i2c_device, 0x05, cmd, 1);
    // 写入数据到寄存器地址 0x00
//    result = BSP_I2C_Master_Transmit(&i2c_device, 0x1B, test_data, 4);
//    if(result == 0)
//    {
////        printf("I2C 写入成功\n");
//    }
//    else
//    {
////        printf("I2C 写入失败\n");
//    }
//    
//    // 延时一段时间
//    Delay(0xFFFF);
    
    // 从寄存器地址 0x00 读取数据
    result = BSP_I2C_Master_Receive(&i2c_device, 0x75, read_data, 4);
    if(result == 0)
    {
//        printf("I2C 读取成功: 0x%02X 0x%02X 0x%02X 0x%02X\n", 
//               read_data[0], read_data[1], read_data[2], read_data[3]);
    }
    else
    {
//        printf("I2C 读取失败\n");
    }
    // 扫描I2C总线上的设备
    BSP_I2C_ScanDevices(&i2c_device, found_devices, 120);
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
    // LedInit(PORT_GROUP1, LED1_PIN);
    // LedInit(PORT_GROUP2, LED2_PIN);
    BSP_GPIO_Init();    


    BSP_GPIO_WritePin(R_LED_GPIO, R_LED_PIN, GPIO_PIN_SET);
    /* 点亮Led1 */
    // LedOn(PORT_GROUP1, LED1_PIN);

     I2C_Polling_Test();

    while (1)
    {
        /* LED1和LED2在同一个端口组，通过异或操作使Led2闪烁，不影响Led1 */
        // LedBlink(PORT_GROUP1, LED1_PIN);
//        BSP_GPIO_TogglePin(G_LED_GPIO, G_LED_PIN);
//        BSP_GPIO_TogglePin(R_LED_GPIO, R_LED_PIN);
        /* LED3、LED4和LED5在同一个端口组 */
        /* 通过PBC寄存器关闭Led3和Led4，不影响同一端口组的其他引脚 */
        // LedOff(PORT_GROUP2, LED2_PIN);
        BSP_GPIO_WritePin(R_LED_GPIO, R_LED_PIN, GPIO_PIN_SET);
        /* 插入延时 */
        Delay(0x28FFFF);


        // LedOnOff(PORT_GROUP2, (LED2_PIN << 16));
        BSP_GPIO_WritePin(R_LED_GPIO, R_LED_PIN, GPIO_PIN_RESET);
        /* 插入延时 */
        Delay(0x28FFFF);

        /* 点亮Led3 */
        // LedOn(PORT_GROUP2, LED2_PIN);
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