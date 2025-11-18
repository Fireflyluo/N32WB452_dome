# N32WB452 rtt 工程说明

## 简介

本工程是N32WB452的RT-thread标准版样例工程，其搭载的MCU主要资源参数如下：

| 硬件      | 描述          |
| --------- | ------------- |
| 芯片型号  | N32WB452CEQ6  |
| CPU       | ARM Cortex M4 |
| 主频      | 144M          |
| 片内SRAM  | 144K          |
| 片内FLASH | 512K          |

## 编译说明

本工程支持rtt的Env工具，可以直接scons编译，或者生成mdk5 iar 工程。

## 板载资源

- MCU：N32WB452CEQ6，主频 144MHz，512KB FLASH
- 常用外设
  - LED ：2个，D0 (PC13)，D1（PA11）
  - 按键：1个，S1(RST)

- 常用接口：串口（USART2、USART3）、i2c、spi

- 调试接口: CMSIS-DAP SWD 

## 外设支持(以下内容为bsp包作者注明，后续会根据我的板载资源调整)

本 BSP 目前对外设驱动的支持情况如下：

| 驱动      | 支持情况  |            备注                      |
| --------- | --------  | -------------------------------------|
| UART      | 支持      | USART1/2/3, UART4/5/6/7              |
| GPIO      | 支持      | PA0, PA1... PE15 ---> PIN: 0, 1...79 |
| I2C       | 支持      | 软件I2C                              |
| SPI       | 支持      | SPI1/2/3                             |
| ADC       | 支持      | ADC1/2                               |
| CAN       | 支持      | CAN1/2                               |
| DAC       | 支持      | DAC1/2                               |
| HWTIMER   | 支持      | TIMER1/2/3/4/5/6/7/8                 |
| WDT       | 支持      | IWDG                                 |
| RTC       | 支持      | 时钟源支持LSE/LSI/HSE                | 

### IO在板级支持包中的映射情况

| IO号 | 板级包中的定义 |
| ---- | -------------- |
| PA9  | USART1_TX      |
| PA10 | USART1_RX      |

| PA2  | USART2_TX      |
| PA3  | USART2_RX      |

| PB10 | USART3_TX      |
| PB11 | USART3_RX      |

| PA13 | UART4_TX       |
| PA14 | UART4_RX       |

| PB13 | UART5_TX       |
| PB14 | UART5_RX       |

| PB0  | UART6_TX       |
| PB1  | UART6_RX       |

| PC2  | UART7_TX       |
| PC3  | UART7_RX       |

| PA4  | SPI1_NSS       |
| PA5  | SPI1_SCK       |
| PA6  | SPI1_MISO      |
| PA7  | SPI1_MOSI      |

| PB12 | SPI2_NSS       |
| PB13 | SPI2_SCK       |
| PB14 | SPI2_MISO      |
| PB15 | SPI2_MOSI      |

| PA15 | SPI3_NSS       |
| PB3  | SPI3_SCK       |
| PB4  | SPI3_MISO      |
| PB5  | SPI3_MOSI      |

| PA1  | ADC1_IN2       |
| PA3  | ADC1_IN4       |

| PA4  | ADC2_IN1       |
| PA5  | ADC2_IN2       |

| PA4  | DAC_OUT1       |
| PA5  | DAC_OUT2       |

| PB8  | CAN1_RX        |
| PB9  | CAN1_TX        |

| PB12 | CAN2_RX        |
| PB13 | CAN2_TX        |

## 使用说明

本章节是为刚接触 RT-Thread 的新手准备的使用说明，遵循简单的步骤即可将 RT-Thread 操作系统运行在该开发板上，看到实验效果 。

### 快速上手


#### 硬件连接

使用数据线连接开发板到 PC，打开电源开关。

#### 编译下载

双击 project.uvprojx 文件，打开 MDK5 工程，编译并下载程序到开发板。

#### 运行结果

下载程序成功之后，系统会自动运行，观察开发板上 LED 的运行效果，D0 D1 会周期性闪烁。

连接开发板对应串口到 PC , 在终端工具里打开相应的串口（115200-8-1-N），复位设备后，在串口上可以看到 RT-Thread 的输出信息:

```
 \ | /
- RT -     Thread Operating System
 / | \     4.1.1 build Apr 24 2022 17:24:22
 2006 - 2022 Copyright by RT-Thread team
msh >
```

## 注意事项
暂无
