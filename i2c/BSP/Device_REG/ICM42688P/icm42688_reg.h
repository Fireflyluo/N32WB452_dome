#ifndef __ICM42688_REG_H__
#define __ICM42688_REG_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif
/* ========================================================================== */
/*                          寄存器Bank选择定义                                */
/* ========================================================================== */
/**
 * @brief Bank选择寄存器 (所有Bank均可访问)
 */
#define ICM42688_REG_BANK_SEL 0x76

    typedef union
    {
        struct
        {
            uint8_t USER_BANK : 3; // [1:0] 用户Bank选择
            uint8_t RESERVED : 5;  // [7:2] 保留
        } bits;
        uint8_t reg;
    } icm42688_reg_bank_sel_t;

    /* Bank选择枚举 */
    typedef enum
    {
        ICM42688_BANK0 = 0,
        ICM42688_BANK1 = 1,
        ICM42688_BANK2 = 2,
        ICM42688_BANK3 = 3, // 注：Bank3在文档中未详细描述
        ICM42688_BANK4 = 4,
        ICM42688_BANK_MAX = 7
    } icm42688_bank_t;

#define ICM42688P_DEVICE_ID 0x68U  ///< WHO_AM_I register value
#define ICM42688P_I2C_ADDR_0 0x68U ///< SDO = GND
#define ICM42688P_I2C_ADDR_1 0x69U ///< SDO = VDDIO

/* ========================================================================== */
/*                          User Bank 0 寄存器定义                           */
/* ========================================================================== */
// === Bank 0 Registers ===
/* Bank 0 设备配置寄存器 */
#define ICM42688_REG_DEVICE_CONFIG 0x11
#define ICM42688_REG_DRIVE_CONFIG 0x12
#define ICM42688_REG_INT_CONFIG 0x14

/* Bank 0 传感器数据寄存器 */
#define ICM42688_REG_TEMP_DATA1 0x1D
#define ICM42688_REG_TEMP_DATA0 0x1E
#define ICM42688_REG_ACCEL_DATA_X1 0x1F
#define ICM42688_REG_ACCEL_DATA_X0 0x20
#define ICM42688_REG_ACCEL_DATA_Y1 0x21
#define ICM42688_REG_ACCEL_DATA_Y0 0x22
#define ICM42688_REG_ACCEL_DATA_Z1 0x23
#define ICM42688_REG_ACCEL_DATA_Z0 0x24
#define ICM42688_REG_GYRO_DATA_X1 0x25
#define ICM42688_REG_GYRO_DATA_X0 0x26
#define ICM42688_REG_GYRO_DATA_Y1 0x27
#define ICM42688_REG_GYRO_DATA_Y0 0x28
#define ICM42688_REG_GYRO_DATA_Z1 0x29
#define ICM42688_REG_GYRO_DATA_Z0 0x2A

/* Bank 0 时间戳和中断状态寄存器 */
#define ICM42688_REG_TMST_FSYNCH 0x2B
#define ICM42688_REG_TMST_FSYNCL 0x2C
#define ICM42688_REG_INT_STATUS 0x2D

/* Bank 0 FIFO相关寄存器 */
#define ICM42688_REG_FIFO_COUNTH 0x2E
#define ICM42688_REG_FIFO_COUNTL 0x2F
#define ICM42688_REG_FIFO_DATA 0x30

/* Bank 0 APEX数据寄存器 */
#define ICM42688_REG_APEX_DATA0 0x31
#define ICM42688_REG_APEX_DATA1 0x32
#define ICM42688_REG_APEX_DATA2 0x33
#define ICM42688_REG_APEX_DATA3 0x34
#define ICM42688_REG_APEX_DATA4 0x35
#define ICM42688_REG_APEX_DATA5 0x36

/* Bank 0 中断状态寄存器 */
#define ICM42688_REG_INT_STATUS2 0x37
#define ICM42688_REG_INT_STATUS3 0x38

/* Bank 0 信号路径复位寄存器 */
#define ICM42688_REG_SIGNAL_PATH_RESET 0x4B

/* Bank 0 接口配置寄存器 */
#define ICM42688_REG_INTF_CONFIG0 0x4C
#define ICM42688_REG_INTF_CONFIG1 0x4D

/* Bank 0 电源管理寄存器 */
#define ICM42688_REG_PWR_MGMT0 0x4E
// #define ICM42688_REG_PWR_MGMT1 0x4F // 注：文档中未详细描述
// #define ICM42688_REG_PWR_MGMT2 0x50 // 注：文档中未详细描述

/* Bank 0 陀螺仪配置寄存器 */
#define ICM42688_REG_GYRO_CONFIG0 0x4F
#define ICM42688_REG_GYRO_CONFIG1 0x51
#define ICM42688_REG_GYRO_ACCEL_CONFIG0 0x52

/* Bank 0 加速度计配置寄存器 */
#define ICM42688_REG_ACCEL_CONFIG0 0x50
#define ICM42688_REG_ACCEL_CONFIG1 0x53

/* Bank 0 时间戳配置寄存器 */
#define ICM42688_REG_TMST_CONFIG 0x54

/* Bank 0 APEX配置寄存器 */
#define ICM42688_REG_APEX_CONFIG0 0x55
#define ICM42688_REG_SMD_CONFIG 0x56 // 注：文档中未详细描述

/* Bank 0 FIFO配置寄存器 */
#define ICM42688_REG_FIFO_CONFIG1 0x5F
#define ICM42688_REG_FIFO_CONFIG2 0x60
#define ICM42688_REG_FIFO_CONFIG3 0x61

/* Bank 0 FSYNC配置寄存器 */
#define ICM42688_REG_FSYNC_CONFIG 0x62

/* Bank 0 中断配置寄存器 */
#define ICM42688_REG_INT_CONFIG0 0x63
#define ICM42688_REG_INT_CONFIG1 0x64

/* Bank 0 中断源寄存器 */
#define ICM42688_REG_INT_SOURCE0 0x65
#define ICM42688_REG_INT_SOURCE1 0x66
#define ICM42688_REG_INT_SOURCE3 0x68 // 注：INT_SOURCE2未定义
#define ICM42688_REG_INT_SOURCE4 0x69
#define ICM42688_REG_INT_SOURCE5 0x6A // 注：文档中未详细描述

/* Bank 0 FIFO丢失包计数寄存器 */
#define ICM42688_REG_FIFO_LOST_PKT0 0x6B
#define ICM42688_REG_FIFO_LOST_PKT1 0x6C

/* Bank 0 自测配置寄存器 */
#define ICM42688_REG_SELF_TEST_CONFIG 0x70

/* Bank 0 设备ID寄存器 */
#define ICM42688_REG_WHO_AM_I 0x75

/* ========================================================================== */
/*                          User Bank 1 寄存器定义                           */
/* ========================================================================== */

/* Bank 1 传感器配置寄存器 */
#define ICM42688_REG_SENSOR_CONFIG0 0x03

/* Bank 1 陀螺仪静态配置寄存器 */
#define ICM42688_REG_GYRO_CONFIG_STATIC2 0x0B
#define ICM42688_REG_GYRO_CONFIG_STATIC3 0x0C
#define ICM42688_REG_GYRO_CONFIG_STATIC4 0x0D
#define ICM42688_REG_GYRO_CONFIG_STATIC5 0x0E
#define ICM42688_REG_GYRO_CONFIG_STATIC6 0x0F
#define ICM42688_REG_GYRO_CONFIG_STATIC7 0x10
#define ICM42688_REG_GYRO_CONFIG_STATIC8 0x11
#define ICM42688_REG_GYRO_CONFIG_STATIC9 0x12
#define ICM42688_REG_GYRO_CONFIG_STATIC10 0x13

/* Bank 1 陀螺仪自测数据寄存器 */
#define ICM42688_REG_XG_ST_DATA 0x5F
#define ICM42688_REG_YG_ST_DATA 0x60
#define ICM42688_REG_ZG_ST_DATA 0x61

/* Bank 1 时间戳值寄存器 */
#define ICM42688_REG_TMSTVAL0 0x62
#define ICM42688_REG_TMSTVAL1 0x63
#define ICM42688_REG_TMSTVAL2 0x64

/* Bank 1 接口配置寄存器 */
#define ICM42688_REG_INTF_CONFIG4 0x7A
#define ICM42688_REG_INTF_CONFIG5 0x7B
#define ICM42688_REG_INTF_CONFIG6 0x7C
/* ========================================================================== */
/*                          User Bank 2 寄存器定义                           */
/* ========================================================================== */

/* Bank 2 加速度计静态配置寄存器 */
#define ICM42688_REG_ACCEL_CONFIG_STATIC2 0x03
#define ICM42688_REG_ACCEL_CONFIG_STATIC3 0x04
#define ICM42688_REG_ACCEL_CONFIG_STATIC4 0x05

/* Bank 2 加速度计自测数据寄存器 */
#define ICM42688_REG_XA_ST_DATA 0x3B
#define ICM42688_REG_YA_ST_DATA 0x3C
#define ICM42688_REG_ZA_ST_DATA 0x3D

/* ========================================================================== */
/*                          User Bank 4 寄存器定义                           */
/* ========================================================================== */

/* Bank 4 APEX配置寄存器 */
#define ICM42688_REG_APEX_CONFIG1 0x40
#define ICM42688_REG_APEX_CONFIG2 0x41
#define ICM42688_REG_APEX_CONFIG3 0x42
#define ICM42688_REG_APEX_CONFIG4 0x43
#define ICM42688_REG_APEX_CONFIG5 0x44
#define ICM42688_REG_APEX_CONFIG6 0x45
#define ICM42688_REG_APEX_CONFIG7 0x46
#define ICM42688_REG_APEX_CONFIG8 0x47
#define ICM42688_REG_APEX_CONFIG9 0x48

/* Bank 4 加速度计WOM阈值寄存器 */
#define ICM42688_REG_ACCEL_WOM_X_THR 0x4A
#define ICM42688_REG_ACCEL_WOM_Y_THR 0x4B
#define ICM42688_REG_ACCEL_WOM_Z_THR 0x4C

/* Bank 4 中断源寄存器 */
#define ICM42688_REG_INT_SOURCE6 0x4D
#define ICM42688_REG_INT_SOURCE7 0x4E
#define ICM42688_REG_INT_SOURCE8 0x4F
#define ICM42688_REG_INT_SOURCE9 0x50
#define ICM42688_REG_INT_SOURCE10 0x51

/* Bank 4 用户偏移寄存器 */
#define ICM42688_REG_OFFSET_USER0 0x77
#define ICM42688_REG_OFFSET_USER1 0x78
#define ICM42688_REG_OFFSET_USER2 0x79
#define ICM42688_REG_OFFSET_USER3 0x7A
#define ICM42688_REG_OFFSET_USER4 0x7B
#define ICM42688_REG_OFFSET_USER5 0x7C
#define ICM42688_REG_OFFSET_USER6 0x7D
#define ICM42688_REG_OFFSET_USER7 0x7E
#define ICM42688_REG_OFFSET_USER8 0x7F

    /* ========================================================================== */
    /*                          重要寄存器位域定义                               */
    /* ========================================================================== */
    /**
     * @brief 设备配置寄存器 (Bank 0, Addr 0x11)
     */
    typedef union
    {
        struct
        {
            uint8_t SOFT_RESET : 1; // [0] 软件复位
            uint8_t RESERVED0 : 3;  // [3:1] 保留
            uint8_t ACCEL_MODE : 2; // [5:4] 加速度计模式
            uint8_t GYRO_MODE : 2;  // [7:6] 陀螺仪模式
        } bits;
        uint8_t reg;
    } icm42688_reg_device_config_t;

    /**
     * @brief 电源管理寄存器0 (Bank 0, Addr 0x4E)
     */
    typedef union
    {
        struct
        {
            uint8_t TEMP_DIS : 1;   // [0] 温度传感器禁用
            uint8_t IDLE : 1;       // [1] 空闲模式
            uint8_t GYRO_MODE : 2;  // [3:2] 陀螺仪模式
            uint8_t ACCEL_MODE : 2; // [5:4] 加速度计模式
            uint8_t RESERVED : 2;   // [7:6] 保留
        } bits;
        uint8_t reg;
    } icm42688_reg_pwr_mgmt0_t;

    /**
     * @brief 陀螺仪配置寄存器0 (Bank 0, Addr 0x4F)
     */
    typedef union
    {
        struct
        {
            uint8_t GYRO_ODR : 4;       // [3:0] 陀螺仪输出数据速率
            uint8_t GYRO_UI_FS_SEL : 3; // [6:4] 陀螺仪满量程选择
            uint8_t RESERVED : 1;       // [7] 保留
        } bits;
        uint8_t reg;
    } icm42688_reg_gyro_config0_t;

    /**
     * @brief 加速度计配置寄存器0 (Bank 0, Addr 0x50)
     */
    typedef union
    {
        struct
        {
            uint8_t ACCEL_ODR : 4;       // [3:0] 加速度计输出数据速率
            uint8_t ACCEL_UI_FS_SEL : 3; // [6:4] 加速度计满量程选择
            uint8_t RESERVED : 1;        // [7] 保留
        } bits;
        uint8_t reg;
    } icm42688_reg_accel_config0_t;
    /**
     * @brief 陀螺仪配置寄存器1 (Bank 0, Addr 0x51)
     * @note 用于配置陀螺仪UI滤波器参数
     */
    typedef union
    {
        struct
        {
            uint8_t GYRO_UI_FILT_ORD : 2; // [1:0] 陀螺仪UI滤波器阶数
            uint8_t GYRO_UI_FILT_BW : 3;  // [4:2] 陀螺仪UI滤波器带宽
            uint8_t RESERVED : 3;         // [7:5] 保留位
        } bits;
        uint8_t reg;
    } icm42688_reg_gyro_config1_t;
    /**
     * @brief 加速度计配置寄存器1 (Bank 0, Addr 0x53)
     * @note 用于配置加速度计UI滤波器参数
     */
    typedef union
    {
        struct
        {
            uint8_t ACCEL_UI_FILT_ORD : 2; // [1:0] 加速度计UI滤波器阶数
            uint8_t ACCEL_UI_FILT_BW : 3;  // [4:2] 加速度计UI滤波器带宽
            uint8_t RESERVED : 3;          // [7:5] 保留位
        } bits;
        uint8_t reg;
    } icm42688_reg_accel_config1_t;
    /**
     * @brief 时间戳配置寄存器 (Bank 0, Addr 0x54)
     * @note 用于配置时间戳功能和时钟源选择
     */
    typedef union
    {
        struct
        {
            uint8_t TMST_EN : 1;     // [0] 时间戳使能位
            uint8_t TMST_SOURCE : 2; // [2:1] 时间戳源选择
            uint8_t RESERVED : 5;    // [7:3] 保留位
        } bits;
        uint8_t reg;
    } icm42688_reg_tmst_config_t;

    /**
     * @brief FIFO配置寄存器1 (Bank 0, Addr 0x5F)
     */
    typedef union
    {
        struct
        {
            uint8_t FIFO_MODE : 1;     // [0] FIFO模式
            uint8_t FIFO_ACCEL_EN : 1; // [1] 加速度计FIFO使能
            uint8_t FIFO_GYRO_EN : 1;  // [2] 陀螺仪FIFO使能
            uint8_t FIFO_TEMP_EN : 1;  // [3] 温度FIFO使能
            uint8_t RESERVED : 4;      // [7:4] 保留
        } bits;
        uint8_t reg;
    } icm42688_reg_fifo_config1_t;

    /**
     * @brief 中断配置寄存器0 (Bank 0, Addr 0x63)
     */
    typedef union
    {
        struct
        {
            uint8_t INT1_MODE : 1;     // [0] INT1引脚模式
            uint8_t INT1_DRIVE : 1;    // [1] INT1驱动电路
            uint8_t INT1_POLARITY : 1; // [2] INT1极性
            uint8_t INT2_MODE : 1;     // [3] INT2引脚模式
            uint8_t INT2_DRIVE : 1;    // [4] INT2驱动电路
            uint8_t INT2_POLARITY : 1; // [5] INT2极性
            uint8_t RESERVED : 2;      // [7:6] 保留
        } bits;
        uint8_t reg;
    } icm42688_reg_int_config0_t;

    /**
     * @brief 中断配置寄存器1 (Bank 0, Addr 0x64)
     */
    typedef union
    {
        struct
        {
            uint8_t INT_TPULSE_DURATION : 4; // [3:0] 中断脉冲持续时间
            uint8_t INT_ASYNC_RESET : 1;     // [4] 异步复位
            uint8_t RESERVED : 3;            // [7:5] 保留
        } bits;
        uint8_t reg;
    } icm42688_reg_int_config1_t;
    /**
     * @brief 自检配置寄存器 (Bank 0, Addr 0x70)
     * @note 用于控制陀螺仪和加速度计的自检功能
     */
    typedef union
    {
        struct
        {
            uint8_t GYRO_SELF_TEST_EN : 1;  // [0] 陀螺仪自检使能
            uint8_t ACCEL_SELF_TEST_EN : 1; // [1] 加速度计自检使能
            uint8_t RESERVED : 6;           // [7:2] 保留位
        } bits;
        uint8_t reg;
    } icm42688_reg_self_test_config_t;
    /**
     * @brief 接口配置寄存器0 (Bank 0, Addr 0x4C)
     */
    typedef union
    {
        struct
        {
            uint8_t FIFO_HOLD_LAST_DATA_EN : 1; // [7] FIFO保持最后数据使能
            uint8_t RESERVED : 7;               // [6:0] 保留
        } bits;
        uint8_t reg;
    } icm42688_reg_intf_config0_t;

    /**
     * @brief 接口配置寄存器1 (Bank 0, Addr 0x4D)
     */
    typedef union
    {
        struct
        {
            uint8_t RTC_MODE : 1; // [2] RTC模式
            uint8_t RESERVED : 7; // [7:3][1:0] 保留
        } bits;
        uint8_t reg;
    } icm42688_reg_intf_config1_t;
    /* ========================================================================== */
    /*                          传感器参数配置枚举                               */
    /* ========================================================================== */

    /**
     * @brief 传感器模式配置
     */
    typedef enum
    {
        ICM42688_MODE_OFF = 0,       // 关闭模式
        ICM42688_MODE_LOW_NOISE = 1, // 低噪声模式
        ICM42688_MODE_LOW_POWER = 2, // 低功耗模式
    } icm42688_sensor_mode_t;

    /**
     * @brief 陀螺仪满量程范围
     */
    typedef enum
    {
        ICM42688_GYRO_FS_2000DPS = 0,   // ±2000 dps
        ICM42688_GYRO_FS_1000DPS = 1,   // ±1000 dps
        ICM42688_GYRO_FS_500DPS = 2,    // ±500 dps
        ICM42688_GYRO_FS_250DPS = 3,    // ±250 dps
        ICM42688_GYRO_FS_125DPS = 4,    // ±125 dps
        ICM42688_GYRO_FS_62_5DPS = 5,   // ±62.5 dps
        ICM42688_GYRO_FS_31_25DPS = 6,  // ±31.25 dps
        ICM42688_GYRO_FS_15_625DPS = 7, // ±15.625 dps
    } icm42688_gyro_fs_t;

    /**
     * @brief 加速度计满量程范围
     */
    typedef enum
    {
        ICM42688_ACCEL_FS_16G = 0, // ±16g
        ICM42688_ACCEL_FS_8G = 1,  // ±8g
        ICM42688_ACCEL_FS_4G = 2,  // ±4g
        ICM42688_ACCEL_FS_2G = 3,  // ±2g
    } icm42688_accel_fs_t;

    /**
     * @brief 输出数据速率配置
     */
    typedef enum
    {
        ICM42688_ODR_32000HZ = 1,   // 32 kHz
        ICM42688_ODR_16000HZ = 2,   // 16 kHz
        ICM42688_ODR_8000HZ = 3,    // 8 kHz
        ICM42688_ODR_4000HZ = 4,    // 4 kHz
        ICM42688_ODR_2000HZ = 5,    // 2 kHz
        ICM42688_ODR_1000HZ = 6,    // 1 kHz
        ICM42688_ODR_500HZ = 7,     // 500 Hz
        ICM42688_ODR_200HZ = 8,     // 200 Hz
        ICM42688_ODR_100HZ = 9,     // 100 Hz
        ICM42688_ODR_50HZ = 10,     // 50 Hz
        ICM42688_ODR_25HZ = 11,     // 25 Hz
        ICM42688_ODR_12_5HZ = 12,   // 12.5 Hz
        ICM42688_ODR_6_25HZ = 13,   // 6.25 Hz
        ICM42688_ODR_3_125HZ = 14,  // 3.125 Hz
        ICM42688_ODR_1_5625HZ = 15, // 1.5625 Hz
    } icm42688_odr_t;

    /**
     * @brief 滤波器配置
     */
    typedef enum
    {
        ICM42688_FILTER_ORDER_FIRST = 0,  // 一阶滤波器
        ICM42688_FILTER_ORDER_SECOND = 1, // 二阶滤波器
        ICM42688_FILTER_ORDER_THIRD = 2,  // 三阶滤波器
    } icm42688_filter_order_t;

    /**
     * @brief 滤波器带宽配置
     */
    typedef enum
    {
        ICM42688_FILTER_BW_ODR_DIV_2 = 0,  // ODR/2
        ICM42688_FILTER_BW_ODR_DIV_4 = 1,  // ODR/4
        ICM42688_FILTER_BW_ODR_DIV_5 = 2,  // ODR/5
        ICM42688_FILTER_BW_ODR_DIV_10 = 3, // ODR/10
        // ... 更多带宽选项根据实际应用需求添加
    } icm42688_filter_bw_t;
    /**
     * @brief 时间戳源选择枚举
     * @note 定义时间戳的时钟源类型
     */
    typedef enum
    {
        ICM42688_TMST_SOURCE_INTERNAL = 0, // 内部时钟源
        ICM42688_TMST_SOURCE_EXTERNAL = 1, // 外部时钟输入
        ICM42688_TMST_SOURCE_FSYNC = 2,    // FSYNC信号作为时间戳源
    } icm42688_timestamp_source_t;
    /**
     * @brief 中断引脚配置
     */
    typedef enum
    {
        ICM42688_INT_PUSH_PULL = 0,  // 推挽输出
        ICM42688_INT_OPEN_DRAIN = 1, // 开漏输出
    } icm42688_int_drive_t;

    typedef enum
    {
        ICM42688_INT_ACTIVE_HIGH = 0, // 高电平有效
        ICM42688_INT_ACTIVE_LOW = 1,  // 低电平有效
    } icm42688_int_polarity_t;

    typedef enum
    {
        ICM42688_INT_LEVEL = 0, // 电平模式
        ICM42688_INT_PULSE = 1, // 脉冲模式
    } icm42688_int_mode_t;

    /**
     * @brief FIFO模式配置
     */
    typedef enum
    {
        ICM42688_FIFO_MODE_STREAM = 0,   // 流模式
        ICM42688_FIFO_MODE_SNAPSHOT = 1, // 快照模式
    } icm42688_fifo_mode_t;

    /**
     * @brief APEX功能配置
     */
    typedef enum
    {
        ICM42688_APEX_PEDOMETER = 0,          // 计步器
        ICM42688_APEX_TILT_DETECTION = 1,     // 倾斜检测
        ICM42688_APEX_RAISE_TO_WAKE = 2,      // 抬手唤醒
        ICM42688_APEX_TAP_DETECTION = 3,      // 敲击检测
        ICM42688_APEX_WAKE_ON_MOTION = 4,     // 运动唤醒
        ICM42688_APEX_SIGNIFICANT_MOTION = 5, // 显著运动检测
    } icm42688_apex_feature_t;

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_REGS_H */