# N32WB452 I2C通信工程

本工程是基于N32WB452微控制器的基础I2C示例工程。


## 项目概述

这是一个简单的I2C示例程序，展示N32WB452的基本硬件i2c操作。



## 主要特性

- BSP层 - 硬件抽象层，提供I2C操作接口
- I2C扫描函数
- I2C轮询收发函数

## 硬件配置

- i2c1 SDA 接到 GPIOB_PIN_7
- i2c1 SCL 接到 GPIOB_PIN_6

- i2c2 SDA 接到 GPIOB_PIN_11
- i2c2 SCL 接到 GPIOB_PIN_10

## 软件架构

### 核心组件
1. `main.c` - 主程序文件，包含LED控制逻辑
2. `main.h` - 头文件，定义引脚配置和相关常量
3. 标准外设驱动库 - 提供GPIO和RCC等外设控制接口


## 使用方法

1. 使用Keil MDK或EIDE打开对应工程
2. 编译并下载程序到N32WB452开发板


## API说明

