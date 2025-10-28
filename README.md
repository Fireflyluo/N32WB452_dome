# N32WB452_dome 验证板项目

## 项目概述

本项目是基于自制的 N32WB452 的学习记录和驱动示例。该验证板集成了多种外设和传感器，用于学习和验证 N32WB452 的各项功能。

## 硬件资源

验证板搭载的主要硬件资源包括：

- **主控芯片**: N32WB452ceq6
- **传感器**:
  - 陀螺仪（型号: ICM42688P ）
  - 气压计（型号: IPC20100 ）
  - 数字温度计（型号: T117 ）
  - 磁力计（型号: MMC5603NJ ）
  - 血氧传感器（型号: GH3018 ）
  - 离线语音识别芯片（型号: CI1306 ）
- **存储**:
  - W25Q128 Flash 存储器
- **显示**:
  - LCD 屏幕
- **其他外设**:
  - LED 指示灯

## 项目结构

```
├── N32wb452_Project    -- LED闪烁示例工程
│   └── ...             
└── Readme.md         --文档
```
## 工程结构示例
```
├── Application                     # 应用层 (业务逻辑、任务调度、UI交互)
│   
├── BSP                             # 板级支持包 (硬件直接操作层)
│   ├── MCU_Peripheral              # 片上外设驱动 (GPIO/SPI/I2C等初始化与读写)
│   |   ├── bsp_i2c.c               # I2C初始化与读写
│   |   ├── bsp_uart.c              # UART初始化与读写
│   │   └── bsp_gpio.c              # GPIO初始化与读写
│   └── Device_REG                  # 设备寄存器操作
│       ├── lcd_reg.c               # LCD寄存器级操作
│       └── icm42688                # IMU寄存器级操作
├── Drivers                         # 设备驱动层 (硬件抽象接口)
│   ├── Sensor                      # 传感器标准化接口 
│   │   ├── lcd.c                   # LCD的init/read/write接口
│   │   └──icm42688.c               # IMU的init/read/write接口
│   └── Sensor_Factory              # 工厂模式抽象层 (统一传感器访问接口)
├── Libraries                       # 芯片厂商提供的底层库 
│   ├── CMSIS                       # ARM内核抽象层 (如STM32的CMSIS)
│   ├── n32wb452_algo_lib           # 算法库 (加密算法、信号处理等)
│   ├── n32wb452_ble_driver         # BLE蓝牙驱动库
│   ├── n32wb452_std_periph_driver  # 标准外设驱动库 (GPIO, UART, SPI, I2C等)
│   └── n32wb452_usbfs_driver       # USB全速驱动库 (USB设备/主机模式支持)
├── Middleware                      # 通用中间件 (硬件无关)
│   ├── Algorithm                   # 算法库 (crc, filter, math)
│   ├── Data_Structure              # 数据结构 (ringbuff, queue, list)
│   └── Utilities                   # 工具函数 (printf重定向, 位操作)
├── Platform                        # 平台抽象层 (可选)
│   ├── platform.c                  # 系统时钟/延时抽象
│   └── gpio_abstract.h             # GPIO操作抽象接口
├── Project                         # 工程文件 (IDE相关)
│   ├── eide                        # EIDE工程
│   └── MDK-ARM                     # Keil工程
└── Include                         # 全局头文件 
    ├── config.h                    # 项目配置
    ├── ioconfig.h                  # 引脚配置
    └── sensor_defines.h            # 传感器通用参数定义

```

## 开发环境


- **IDE/编译器**: keil/vscode eide
- **编程语言**: C
- **调试工具**: dap-link
- **依赖库**: 标准库

## 快速开始

1. **克隆项目**
   ```bash
   git clone https://github.com/Fireflyluo/N32WB452_dome.git
   cd N32WB452_dome
   ```

2. **导入工程**
   - 使用keil/eide 打开项目目录
   - 选择对应的工程文件（如: dame.uvprojx）

3. **编译和烧录**
   - 配置正确的目标设备（N32wb452）
   - 编译项目
   - 连接调试器并烧录到验证板

4. **运行示例**
   - 目前可运行的示例：
     - LED闪烁示例（N32wb452_Project目录）
     - I2C通信示例（i2c目录）


## 已实现功能

- [x] LED控制（GPIO）
- [x] I2C通信示例
- [x] 串口通信示例
- [x] Icm42688陀螺仪驱动

## 待实现功能

- [ ] SPI通信示例
- [ ] 定时器应用示例
- [ ] 中断处理示例
- [ ] 低功耗模式示例
- [ ] 更多外设驱动...

## 使用说明

每个工程的详细使用说明请查看对应目录下的README文件：

- [N32wb452_Project说明](N32wb452_Project/readme.md)
- [i2c说明](i2c/readme.md)

## 更新日志

- **2025-10-7**: 初始版本提交
  - 添加LED控制示例
  - 修改工程结构和文档描述
- **2025-10-15**: @i2c 功能提交
  - 添加bsp_i2c驱动
  - 修改工程结构和文档描述
  - 添加i2c轮询模式
- **2025-10-23**: @uart 功能提交
  - 添加bsp_uart驱动
  - 修改工程结构和文档描述
  - 添加uart轮询和中断模式
- **2025-10-28**: @icm42688 驱动提交
  - 添加icm42688陀螺仪驱动

## 联系方式

- 作者: Fireflyluo
- qq: 2161486135
- 邮箱: 2161486135@qq.com

## 注意事项

1. 使用前请确保硬件连接正确
2. 部分功能可能需要额外配置
3. 如有问题请查看具体模块的文档或提交Issue

---

*持续更新中...*
