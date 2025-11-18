#ifndef __ICM42688_HAL_H__
#define __ICM42688_HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "icm42688p_reg.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                          常量定义                                         */
/* ========================================================================== */
/* 超时设置 */
#define ICM42688_DEFAULT_TIMEOUT_MS 1000
#define ICM42688_RESET_DELAY_MS 1000
#define ICM42688_MODE_SWITCH_DELAY_MS 100
#define ICM42688_WAKEUP_DELAY_MS 10
/* 设备识别ID */
#define ICM42688_WHO_AM_I_ID 0x47

    /* ========================================================================== */
    /*                          错误码定义                                       */
    /* ========================================================================== */

    /**
     * @brief 驱动错误码
     */
    typedef enum
    {
        ICM42688_OK = 0,                   // 操作成功
        ICM42688_ERR_COMM = -1,            // 通信错误
        ICM42688_ERR_NOT_READY = -2,       // 设备未就绪
        ICM42688_ERR_INVALID_PARAM = -3,   // 无效参数
        ICM42688_ERR_TIMEOUT = -4,         // 操作超时
        ICM42688_ERR_FIFO_OVERFLOW = -5,   // FIFO溢出
        ICM42688_ERR_SELF_TEST_FAIL = -6,  // 自检失败
        ICM42688_ERR_NOT_INITIALIZED = -7, // 设备未初始化
    } icm42688_err_t;
    /* ========================================================================== */
    /*                          传感器数据输出结构体                                 */
    /* ========================================================================== */

    /**
     * @brief 原始传感器数据（16位）
     */
    typedef struct
    {
        int16_t accel_x;     // X轴加速度原始数据
        int16_t accel_y;     // Y轴加速度原始数据
        int16_t accel_z;     // Z轴加速度原始数据
        int16_t gyro_x;      // X轴陀螺仪原始数据
        int16_t gyro_y;      // Y轴陀螺仪原始数据
        int16_t gyro_z;      // Z轴陀螺仪原始数据
        int16_t temperature; // 温度原始数据
    } icm42688_raw_data_t;

    /**
     * @brief 转换后的传感器数据（物理量）
     */
    typedef struct
    {
        float accel_x;      // X轴加速度 [m/s²]
        float accel_y;      // Y轴加速度 [m/s²]
        float accel_z;      // Z轴加速度 [m/s²]
        float gyro_x;       // X轴角速度 [rad/s]
        float gyro_y;       // Y轴角速度 [rad/s]
        float gyro_z;       // Z轴角速度 [rad/s]
        float temperature;  // 温度 [°C]
        uint32_t timestamp; // 时间戳 [ms]
    } icm42688_sensor_data_t;

    /**
     * @brief FIFO数据包结构
     */
    typedef struct
    {
        uint8_t header;      // FIFO包头
        int16_t accel[3];    // 加速度数据
        int16_t gyro[3];     // 陀螺仪数据
        uint8_t temperature; // 温度数据
        uint32_t timestamp;  // 时间戳
        uint8_t packet_size; // 数据包大小
    } icm42688_fifo_packet_t;

    /**
     * @brief 计步器数据
     */
    typedef struct
    {
        uint32_t step_count;    // 步数计数
        uint16_t step_detected; // 步数检测标志
        uint8_t step_cadence;   // 步频
        uint8_t activity_class; // 活动分类
    } icm42688_pedometer_data_t;

    /**
     * @brief 敲击检测数据
     */
    typedef struct
    {
        uint8_t tap_count;     // 敲击次数
        uint8_t tap_axis;      // 敲击轴
        uint8_t tap_direction; // 敲击方向
    } icm42688_tap_data_t;

    /**
     * @brief 设备状态信息
     */
    typedef struct
    {
        bool sensor_ready;        // 传感器就绪状态
        bool data_ready;          // 数据就绪状态
        bool fifo_overflow;       // FIFO溢出状态
        bool fifo_watermark;      // FIFO水印状态
        uint16_t fifo_count;      // FIFO当前数据计数
        uint8_t interrupt_status; // 中断状态寄存器值
        uint32_t error_count;     // 错误计数
    } icm42688_device_status_t;

    /**
     * @brief 设备校准数据
     */
    typedef struct
    {
        float accel_offset[3]; // 加速度计偏移量
        float gyro_offset[3];  // 陀螺仪偏移量
        float accel_scale[3];  // 加速度计缩放因子
        float gyro_scale[3];   // 陀螺仪缩放因子
        bool calibrated;       // 校准标志
    } icm42688_calibration_data_t;
    /* ========================================================================== */
    /*                          函数指针类型定义                                 */
    /* ========================================================================== */

    typedef icm42688_err_t (*icm42688_write_reg_fn)(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len);
    typedef icm42688_err_t (*icm42688_read_reg_fn)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
    typedef void (*icm42688_delay_ms_fn)(uint32_t ms);
    typedef uint32_t (*icm42688_get_tick_fn)(void);
    typedef void (*lock)(void);
    typedef void (*unlock)(void);

    /**
     * @brief 通信接口类型
     */
    typedef enum
    {
        ICM42688_INTERFACE_SPI = 0, // SPI接口
        ICM42688_INTERFACE_I2C = 1, // I2C接口
        ICM42688_INTERFACE_I3C = 2, // I3C接口
    } icm42688_interface_t;

    /**
     * @brief 通信配置结构体
     */
    typedef struct
    {
        icm42688_interface_t interfaces;  // 通信接口类型
        uint8_t dev_addr;                // 设备地址
        icm42688_write_reg_fn write_reg; // 寄存器写函数
        icm42688_read_reg_fn read_reg;   // 寄存器读函数
        icm42688_delay_ms_fn delay_ms;   // 延时函数
        icm42688_get_tick_fn get_tick;   // 获取系统 tick 函数（可选）
        lock lock;                       // 互斥锁函数（可选）
        unlock unlock;                   // 互斥解锁函数（可选）
    } icm42688_comm_config_t;

    /**
     * @brief ICM42688 HAL 句柄结构体
     */
    typedef struct
    {
        icm42688_comm_config_t config; // 通信配置
        uint8_t current_bank;          // 当前寄存器Bank

        /* 状态信息 */
        icm42688_device_status_t status; // 设备状态
        bool initialized;                // 初始化标志
        uint8_t whoami;                  // 设备ID

        /* 内部数据 */
        uint32_t last_read_time; // 最后读取时间
        uint32_t sample_count;   // 采样计数
    } icm42688_hal_handle_t;

    /* FIFO配置 */
    typedef struct
    {
        bool fifo_accel_en;      // 加速度计FIFO使能
        bool fifo_gyro_en;       // 陀螺仪FIFO使能
        bool fifo_temp_en;       // 温度FIFO使能
        bool fifo_tmst_fsync_ev; // 时间戳/FSYNC 数据包发送⾄ FIFO
        uint16_t fifo_watermark; // FIFO水印
    } icm42688_fifo_config_t;

    /* ========================================================================== */
    /*                          设备初始化和管理函数                             */
    /* ========================================================================== */
    icm42688_err_t icm42688_hal_init(icm42688_hal_handle_t *handle, const icm42688_comm_config_t *comm_config);
    icm42688_err_t icm42688_hal_deinit(icm42688_hal_handle_t *handle);

    icm42688_err_t icm42688_hal_reset(icm42688_hal_handle_t *handle);
    icm42688_err_t icm42688_hal_checkid(icm42688_hal_handle_t *handle);

    /* ========================================================================== */
    /*                          传感器配置函数                                   */
    /* ========================================================================== */
    /* 电源管理 */
    icm42688_err_t icm42688_hal_config_power_mode(icm42688_hal_handle_t *handle,
                                                  icm42688_sensor_mode_t gyro_mode,
                                                  icm42688_sensor_mode_t accel_mode);
    /* 配置陀螺仪和加速度计 */
    icm42688_err_t icm42688_hal_set_gyro_config(icm42688_hal_handle_t *handle,
                                                icm42688_gyro_fs_t fs,
                                                icm42688_odr_t odr);
    icm42688_err_t icm42688_hal_get_gyro_config(icm42688_hal_handle_t *handle,
                                                icm42688_gyro_fs_t *fs,
                                                icm42688_odr_t *odr);
    icm42688_err_t icm42688_hal_set_accel_config(icm42688_hal_handle_t *handle,
                                                 icm42688_accel_fs_t fs,
                                                 icm42688_odr_t odr);
    icm42688_err_t icm42688_hal_get_accel_config(icm42688_hal_handle_t *handle,
                                                 icm42688_accel_fs_t *fs,
                                                 icm42688_odr_t *odr);
    /* ========================================================================== */
    /*                          数据读取函数                                     */
    /* ========================================================================== */

    icm42688_err_t icm42688_hal_read_raw_data(icm42688_hal_handle_t *handle,
                                              icm42688_raw_data_t *raw_data);

    icm42688_err_t icm42688_hal_read_sensor_data(icm42688_hal_handle_t *handle,
                                                 icm42688_sensor_data_t *sensor_data);

    icm42688_err_t icm42688_check_data_ready(icm42688_hal_handle_t *handle, bool *data_ready);
    icm42688_err_t icm42688_wait_data_ready(icm42688_hal_handle_t *handle, uint32_t timeout_ms);

    /* ========================================================================== */
    /*                          FIFO相关函数                                     */
    /* ========================================================================== */

    icm42688_err_t icm42688_hal_config_fifo(icm42688_hal_handle_t *handle,
                                            const icm42688_fifo_config_t *config);
    icm42688_err_t icm42688_hal_get_fifo_packet_count(icm42688_hal_handle_t *handle,
                                                      uint16_t *packet_count);
    icm42688_err_t icm42688_hal_hal_read_fifo(icm42688_hal_handle_t *handle,
                                              icm42688_fifo_packet_t *packets,
                                              uint16_t max_packets,
                                              uint16_t *read_packets);
    icm42688_err_t icm42688_hal_flush_fifo(icm42688_hal_handle_t *handle);
    /* ========================================================================== */
    /*                          工具和状态函数                                   */
    /* ========================================================================== */
    icm42688_err_t icm42688_hal_get_device_status(icm42688_hal_handle_t *handle,
                                              icm42688_device_status_t *status);


    icm42688_err_t icm42688_hal_read_temperature(icm42688_hal_handle_t *handle,
                                             float *temperature);


    icm42688_err_t icm42688_hal_get_device_info(icm42688_hal_handle_t *handle,
                                            char *info, uint32_t len);

    void icm42688_convert_accel_data(int16_t raw_data, icm42688_accel_fs_t fs, float *accel);
    void icm42688_convert_gyro_data(int16_t raw_data, icm42688_gyro_fs_t fs, float *gyro);
    /* ========================================================================== */
    /*                          校准和自检函数                                   */
    /* ========================================================================== */

    icm42688_err_t icm42688_hal_self_test(icm42688_hal_handle_t *handle,
                                      bool *gyro_self_test,
                                      bool *accel_self_test);
    icm42688_err_t icm42688_hal_calibrate_sensor(icm42688_hal_handle_t *handle,
                                             uint32_t samples,
                                             uint32_t timeout_ms);

    /* ========================================================================== */
    /*                          APEX功能函数                                     */
    /* ========================================================================== */
    //待实现

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_HAL_H */
