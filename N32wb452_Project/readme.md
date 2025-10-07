# N32WB452 LED闪烁示例工程

本项目是基于N32WB452微控制器的基础LED控制示例工程，演示了如何使用GPIO控制LED灯的亮灭和闪烁效果。

## 项目概述

这是一个简单的LED控制示例程序，展示了N32WB452微控制器的基本GPIO操作。程序通过配置GPIO引脚为推挽输出模式来控制LED，实现LED的点亮、熄灭和闪烁功能。

## 目录结构

```
.
├── Application     # 应用程序代码
│   ├── Inc         # 头文件
│   │   ├── main.h
│   │   └── n32wb452_it.h
│   └── Src         # 源文件
│       ├── main.c
│       └── n32wb452_it.c
├── Libraries       # 芯片标准外设库和协议栈
│   ├── CMSIS              # Cortex微控制器软件接口标准
│   ├── n32wb452_algo_lib  # 加密算法库
│   ├── n32wb452_ble_driver# BLE协议栈
│   ├── n32wb452_std_periph_driver # 标准外设驱动库
│   └── n32wb452_usbfs_driver      # USB全速驱动库
└── Project         # 工程配置文件
    ├── MDK-ARM     # Keil MDK工程配置
    └── eide        # EIDE工程配置
```

## 主要特性

- GPIO初始化配置
- LED点亮/熄灭控制
- LED闪烁效果实现
- 软件延时函数

## 硬件配置

- LED1连接到GPIOA_PIN_11
- LED2连接到GPIOC_PIN_13

## 软件架构

### 核心组件
1. `main.c` - 主程序文件，包含LED控制逻辑
2. `main.h` - 头文件，定义引脚配置和相关常量
3. 标准外设驱动库 - 提供GPIO和RCC等外设控制接口

### 示例程序功能
1. 初始化GPIO引脚为推挽输出模式
2. 点亮LED1
3. 在主循环中控制LED的亮灭和闪烁：
   - LED1持续闪烁
   - LED2按一定节奏闪烁

## 使用方法

1. 使用Keil MDK或EIDE打开对应工程
2. 编译并下载程序到N32WB452开发板
3. 观察LED闪烁效果

## API说明

### GPIO控制函数
- `LedInit()` - 初始化LED GPIO引脚
- `LedOn()` - 点亮指定LED
- `LedOff()` - 熄灭指定LED
- `LedBlink()` - 切换LED状态（开<->关）
- `Delay()` - 简单的软件延时函数
