#include "icm42688_hal.h"
#include "icm42688p_reg.h"
/* 内部寄存器操作宏 */
#define ICM42688_BANK_SELECT(handle, bank)                                                                 \
    do                                                                                                     \
    {                                                                                                      \
        if ((bank) > ICM42688_BANK_MAX)                                                                    \
        {                                                                                                  \
            return ICM42688_ERR_INVALID_PARAM;                                                             \
        }                                                                                                  \
        icm42688_reg_bank_sel_t bank_sel = {.bits.USER_BANK = (uint8_t)(bank)};                            \
        icm42688_err_t ret = icm42688_write_reg_internal(handle, ICM42688_REG_BANK_SEL, &bank_sel.reg, 1); \
        if (ret == ICM42688_OK)                                                                            \
        {                                                                                                  \
            handle->current_bank = (bank);                                                                 \
        }                                                                                                  \
    } while (0)

#define ICM42688_CHECK_INIT(handle)          \
    if (!handle || !handle->initialized)     \
    {                                        \
        return ICM42688_ERR_NOT_INITIALIZED; \
    }

#define ICM42688_CHECK_PARAM(handle)       \
    if (!handle)                           \
    {                                      \
        return ICM42688_ERR_INVALID_PARAM; \
    }
/* ========================================================================== */
/*                           内部静态函数声明                                */
/* ========================================================================== */
/* 寄存器操作内部函数 */
static icm42688_err_t icm42688_write_reg_internal(icm42688_hal_handle_t *handle,
                                                  uint8_t reg_addr,
                                                  const uint8_t *data,
                                                  uint16_t len);
static icm42688_err_t icm42688_read_reg_internal(icm42688_hal_handle_t *handle,
                                                 uint8_t reg_addr,
                                                 uint8_t *data,
                                                 uint16_t len);
/* 数据转换内部函数 */
static float icm42688_convert_accel_raw_to_ms2(int16_t raw, icm42688_accel_fs_t fs);
static float icm42688_convert_gyro_raw_to_rads(int16_t raw, icm42688_gyro_fs_t fs);
static float icm42688_convert_temp_raw_to_celsius(int16_t raw);

/* 校验和工具函数 */
static uint8_t icm42688_calculate_checksum(const uint8_t *data, uint16_t len);
/* ========================================================================== */
/*                           内部使用的常量数据                              */
/* ========================================================================== */
/**
 * @brief 陀螺仪灵敏度查找表 (LSB/dps)
 * @note 根据满量程范围对应的灵敏度值
 */
static const float gyro_sensitivity_lut[] = {
    [ICM42688_GYRO_FS_2000DPS] = 16.384f,     // ±2000 dps
    [ICM42688_GYRO_FS_1000DPS] = 32.768f,     // ±1000 dps
    [ICM42688_GYRO_FS_500DPS] = 65.536f,      // ±500 dps
    [ICM42688_GYRO_FS_250DPS] = 131.072f,     // ±250 dps
    [ICM42688_GYRO_FS_125DPS] = 262.144f,     // ±125 dps
    [ICM42688_GYRO_FS_62_5DPS] = 524.288f,    // ±62.5 dps
    [ICM42688_GYRO_FS_31_25DPS] = 1048.576f,  // ±31.25 dps
    [ICM42688_GYRO_FS_15_625DPS] = 2097.152f, // ±15.625 dps
};

/**
 * @brief 加速度计灵敏度查找表 (LSB/g)
 * @note 根据满量程范围对应的灵敏度值
 */
static const float accel_sensitivity_lut[] = {
    [ICM42688_ACCEL_FS_16G] = 2048.0f, // ±16g
    [ICM42688_ACCEL_FS_8G] = 4096.0f,  // ±8g
    [ICM42688_ACCEL_FS_4G] = 8192.0f,  // ±4g
    [ICM42688_ACCEL_FS_2G] = 16384.0f, // ±2g
};

/**
 * @brief ODR值对应的实际频率表 (Hz)
 * @note 根据寄存器值映射到实际输出频率
 */
static const uint32_t odr_frequency_lut[] = {
    [ICM42688_ODR_32000HZ] = 32000,
    [ICM42688_ODR_16000HZ] = 16000,
    [ICM42688_ODR_8000HZ] = 8000,
    [ICM42688_ODR_4000HZ] = 4000,
    [ICM42688_ODR_2000HZ] = 2000,
    [ICM42688_ODR_1000HZ] = 1000,
    [ICM42688_ODR_500HZ] = 500,
    [ICM42688_ODR_200HZ] = 200,
    [ICM42688_ODR_100HZ] = 100,
    [ICM42688_ODR_50HZ] = 50,
    [ICM42688_ODR_25HZ] = 25,
    [ICM42688_ODR_12_5HZ] = 12,
    [ICM42688_ODR_6_25HZ] = 6,
    [ICM42688_ODR_3_125HZ] = 3,
    [ICM42688_ODR_1_5625HZ] = 1,
};
/* ========================================================================== */
/*                           内部寄存器操作函数                              */
/* ========================================================================== */
/**
 * @brief 内部寄存器写函数
 * @param handle 设备句柄
 * @param reg_addr 寄存器地址
 * @param data 要写入的数据
 * @param len 数据长度
 * @return 错误码
 * @note 此函数处理Bank切换和通信错误重试
 */
static icm42688_err_t icm42688_write_reg_internal(icm42688_hal_handle_t *handle,
                                                  uint8_t reg_addr,
                                                  const uint8_t *data,
                                                  uint16_t len)
{
    icm42688_err_t ret;
    uint8_t retry_count = 3;

    if (!handle || !handle->config.write_reg)
    {
        return ICM42688_ERR_INVALID_PARAM;
    }

    /* 重试机制，提高通信可靠性 */
    while (retry_count--)
    {
        ret = handle->config.write_reg(handle->config.dev_addr, reg_addr, data, len);
        if (ret == ICM42688_OK)
        {
            break;
        }

        /* 通信失败时延时后重试 */
        if (handle->config.delay_ms)
        {
            handle->config.delay_ms(1);
        }
    }

    return ret;
}
/**
 * @brief 内部寄存器读函数
 * @param handle 设备句柄
 * @param reg_addr 寄存器地址
 * @param data 读取数据缓冲区
 * @param len 数据长度
 * @return 错误码
 * @note 此函数处理Bank切换和通信错误重试
 */
static icm42688_err_t icm42688_read_reg_internal(icm42688_hal_handle_t *handle,
                                                 uint8_t reg_addr,
                                                 uint8_t *data,
                                                 uint16_t len)
{
    icm42688_err_t ret;
    uint8_t retry_count = 3;

    if (!handle || !handle->config.read_reg)
    {
        return ICM42688_ERR_INVALID_PARAM;
    }

    /* 重试机制，提高通信可靠性 */
    while (retry_count--)
    {
        ret = handle->config.read_reg(handle->config.dev_addr, reg_addr, data, len);
        if (ret == ICM42688_OK)
        {
            break;
        }

        /* 通信失败时延时后重试 */
        if (handle->config.delay_ms)
        {

            handle->config.delay_ms(1);
        }
    }

    return ret;
}
/**
 * @brief 切换到指定Bank
 * @param handle 设备句柄
 * @param bank 目标Bank
 * @return 错误码
 * @note 如果已经在目标Bank，则不会重复切换
 */
static icm42688_err_t icm42688_switch_bank(icm42688_hal_handle_t *handle, icm42688_bank_t bank)
{
    if (!handle)
    {
        return ICM42688_ERR_INVALID_PARAM;
    }

    /* 如果已经在目标Bank，直接返回 */
    if (handle->current_bank == bank)
    {
        return ICM42688_OK;
    }

    /* 切换Bank */
    icm42688_reg_bank_sel_t bank_sel = {.bits.USER_BANK = bank};
    icm42688_err_t ret = icm42688_write_reg_internal(handle, ICM42688_REG_BANK_SEL,
                                                     &bank_sel.reg, 1);
    if (ret == ICM42688_OK)
    {
        handle->current_bank = bank;
    }

    return ret;
}
/* 内部锁保护函数 */
static void hal_lock(icm42688_hal_handle_t *handle)
{
    if (handle->config.lock)
    {
        handle->config.lock();
    }
}

static void hal_unlock(icm42688_hal_handle_t *handle)
{
    if (handle->config.unlock)
    {
        handle->config.unlock();
    }
}
/* ========================================================================== */
/*                          设备初始化和管理函数                             */
/* ========================================================================== */
/**
 * @brief 初始化ICM-42688设备
 * @param handle 设备句柄指针
 * @param comm_config 通信配置
 * @return 错误码
 * @note 此函数会执行设备复位、ID验证和默认配置
 */
icm42688_err_t icm42688_hal_init(icm42688_hal_handle_t *handle, const icm42688_comm_config_t *comm_config)
{
    ICM42688_CHECK_PARAM(handle);
    ICM42688_CHECK_PARAM(comm_config);

    /* 检查必要的函数指针 */
    if (!comm_config->write_reg || !comm_config->read_reg || !comm_config->delay_ms || !comm_config->get_tick)
    {
        return ICM42688_ERR_INVALID_PARAM;
    }

    /* 初始化句柄 */
    handle->config = *comm_config;
    handle->current_bank = ICM42688_BANK0;
    handle->initialized = false;

    hal_lock(handle);

    /* 检查设备ID */
    uint8_t whoami;
    icm42688_err_t ret = icm42688_read_reg_internal(handle, ICM42688_REG_WHO_AM_I,
                                                    &whoami, 2);
    if (ret != ICM42688_OK)
        return ret;
    if (whoami != ICM42688_WHO_AM_I_ID)
        return ICM42688_ERR_NOT_READY;

    handle->whoami = whoami;
    /* 执行软件复位 */
    ICM42688_BANK_SELECT(handle, ICM42688_BANK0);
    icm42688_reg_device_config_t dev_config = {
        .bits.SOFT_RESET = 1};
    // dev_config.reg = 0x01;   //等价于
    ret = icm42688_write_reg_internal(handle, ICM42688_REG_DEVICE_CONFIG,
                                      &dev_config.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }
    /* 等待复位完成 */
    if (handle->config.delay_ms)
    {
        handle->config.delay_ms(ICM42688_RESET_DELAY_MS);
    }

    handle->initialized = true;
    hal_unlock(handle);

    return ICM42688_OK;
}

/**
 * @brief 释放ICM-42688设备
 * @param handle 设备句柄指针
 * @return 错误码
 */
icm42688_err_t icm42688_hal_deinit(icm42688_hal_handle_t *handle)
{
    ICM42688_CHECK_INIT(handle);

    /* 关闭所有传感器 */
    icm42688_reg_pwr_mgmt0_t pwr_mgmt0 = {
        .bits.GYRO_MODE = ICM42688_MODE_OFF,
        .bits.ACCEL_MODE = ICM42688_MODE_OFF};

    icm42688_err_t ret = icm42688_write_reg_internal(handle, ICM42688_REG_PWR_MGMT0,
                                                     &pwr_mgmt0.reg, 1);

    /* 重置设备状态 */
    memset(handle, 0, sizeof(icm42688_hal_handle_t));

    return ret;
}

/**
 * @brief 重置ICM-42688设备
 * @param handle 设备句柄指针
 * @return 错误码
 */
icm42688_err_t icm42688_hal_reset(icm42688_hal_handle_t *handle)
{
    ICM42688_CHECK_INIT(handle);
    ICM42688_BANK_SELECT(handle, ICM42688_BANK0);

    icm42688_reg_device_config_t dev_config = {
        .bits.SOFT_RESET = 1};

    return icm42688_write_reg_internal(handle, ICM42688_REG_DEVICE_CONFIG,
                                       &dev_config.reg, 1);
}

/**
 * @brief 检查ICM-42688设备ID
 * @param handle 设备句柄指针
 * @return true表示ID匹配，false表示不匹配或未初始化
 */
icm42688_err_t icm42688_hal_checkid(icm42688_hal_handle_t *handle)
{

    if (!handle || !handle->initialized)
    {
        return false;
    }

    uint8_t whoami;
    icm42688_err_t ret = icm42688_read_reg_internal(handle, ICM42688_REG_WHO_AM_I,
                                                    &whoami, 1);

    return (ret == ICM42688_OK) && (whoami == ICM42688_WHO_AM_I_ID);
}

/* ========================================================================== */
/*                          传感器配置函数                                   */
/* ========================================================================== */

/* 配置陀螺仪和加速度计 */
/**
 * @brief 配置陀螺仪和加速度计
 * @param handle 设备句柄指针
 * @param gyro_fs 陀螺仪 Full-Scale
 * @param gyro_odr 陀螺仪输出数据率
 * @return 错误码
 */
icm42688_err_t icm42688_hal_set_gyro_config(icm42688_hal_handle_t *handle,
                                            icm42688_gyro_fs_t gyro_fs,
                                            icm42688_odr_t gyro_odr)
{
    ICM42688_BANK_SELECT(handle, ICM42688_BANK0);

    /* 配置陀螺仪参数 */
    icm42688_reg_gyro_config0_t gyro_config = {
        .bits.GYRO_ODR = gyro_odr,
        .bits.GYRO_FS_SEL = gyro_fs};

    icm42688_err_t ret = icm42688_write_reg_internal(handle, ICM42688_REG_GYRO_CONFIG0,
                                                     &gyro_config.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    return ret;
}
/**
 * @brief 获取陀螺仪配置
 * @param handle 设备句柄指针
 * @param gyro_fs 输出陀螺仪 Full-Scale
 * @param gyro_odr 输出陀螺仪输出数据率
 * @return 错误码
 */
icm42688_err_t icm42688_hal_get_gyro_config(icm42688_hal_handle_t *handle,
                                            icm42688_gyro_fs_t *gyro_fs,
                                            icm42688_odr_t *gyro_odr)
{
    icm42688_reg_gyro_config0_t gyro_config;
    icm42688_err_t ret = icm42688_read_reg_internal(handle, ICM42688_REG_GYRO_CONFIG0,
                                                    &gyro_config.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }
    *gyro_fs = (icm42688_gyro_fs_t)(gyro_config.bits.GYRO_FS_SEL);
    *gyro_odr = (icm42688_odr_t)(gyro_config.bits.GYRO_ODR);
    return ICM42688_OK;
}

icm42688_err_t icm42688_hal_set_accel_config(icm42688_hal_handle_t *handle,
                                             icm42688_accel_fs_t accel_fs,
                                             icm42688_odr_t accel_odr)
{
    ICM42688_BANK_SELECT(handle, ICM42688_BANK0);

    /* 配置加速度计参数 */
    icm42688_reg_accel_config0_t accel_config = {
        .bits.ACCEL_ODR = accel_odr,
        .bits.ACCEL_FS_SEL = accel_fs};

    icm42688_err_t ret = icm42688_write_reg_internal(handle, ICM42688_REG_ACCEL_CONFIG0,
                                                     &accel_config.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    return ret;
}
icm42688_err_t icm42688_hal_get_accel_config(icm42688_hal_handle_t *handle,
                                             icm42688_accel_fs_t *accel_fs,
                                             icm42688_odr_t *accel_odr)
{

    icm42688_reg_accel_config0_t accel_config;
    icm42688_err_t ret = icm42688_read_reg_internal(handle, ICM42688_REG_ACCEL_CONFIG0,
                                                    &accel_config.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }
    *accel_fs = (icm42688_accel_fs_t)(accel_config.bits.ACCEL_FS_SEL);
    *accel_odr = (icm42688_odr_t)(accel_config.bits.ACCEL_ODR);
    return ICM42688_OK;
}
/* 电源管理 */
icm42688_err_t icm42688_hal_config_power_mode(icm42688_hal_handle_t *handle,
                                              icm42688_sensor_mode_t gyro_mode,
                                              icm42688_sensor_mode_t accel_mode)
{
    /* 配置电源 */
    ICM42688_BANK_SELECT(handle, ICM42688_BANK0);
    icm42688_reg_pwr_mgmt0_t pwr_config = {
        .bits.TEMP_DIS = 0,
        .bits.GYRO_MODE = gyro_mode,
        .bits.ACCEL_MODE = accel_mode};
    icm42688_write_reg_internal(handle, ICM42688_REG_PWR_MGMT0,
                                &pwr_config.reg, 1);
    /* 等待配置生效 */
    if (handle->config.delay_ms)
    {
        handle->config.delay_ms(ICM42688_MODE_SWITCH_DELAY_MS);
    }

    handle->status.sensor_ready = true;

    return ICM42688_OK;
}
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
// 待实现