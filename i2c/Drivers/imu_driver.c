#include "imu_driver.h"

#include "bsp_sys.h"
#include "bsp_i2c.h"
#include "icm42688p.h"
#include "icm42688_reg.h"
#include <stdio.h>


extern BSP_I2C_Device i2c_device;
// I2C通信函数实现
static icm42688_err_t i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr,
                                    const uint8_t *data, uint16_t len)
{
    // 实现I2C写寄存器操作
    i2c_device.device_addr = dev_addr << 1;
    BSP_I2C_Master_Transmit(&i2c_device, reg_addr, (uint8_t *)data, len);
    return ICM42688_OK;
}

static icm42688_err_t i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr,
                                   uint8_t *data, uint16_t len)
{
    i2c_device.device_addr = dev_addr << 1;
    // 实现I2C读寄存器操作
    BSP_I2C_Master_Receive(&i2c_device, reg_addr, data, len);
    return ICM42688_OK;
}

static void delay_ms(uint32_t ms)
{
    BSP_Delay(ms);
}

static uint32_t get_tick(void)
{
    return BSP_GetTick();
}

int imu_init(void)
{
    icm42688_device_t sensor;
    icm42688_comm_config_t comm_config = {
        .interface = ICM42688_INTERFACE_I2C,
        .dev_addr = ICM42688P_I2C_ADDR_1, // SDO接GND
        .write_reg = i2c_write_reg,
        .read_reg = i2c_read_reg,
        .delay_ms = delay_ms,
        .get_tick = get_tick};

    // 1. 初始化设备
    icm42688_err_t ret = icm42688_init(&sensor, &comm_config);
    if (ret != ICM42688_OK)
    {
        printf("ICM42688初始化失败: %s\n", icm42688_get_error_string(ret));
        return -1;
    }

    // 2. 配置传感器参数
    icm42688_sensor_config_t config = {
        .gyro_mode = ICM42688_MODE_LOW_NOISE, // 低噪声模式
        .gyro_fs = ICM42688_GYRO_FS_1000DPS,  // ±1000 dps
        .gyro_odr = ICM42688_ODR_1000HZ,      // 1000 Hz ODR

        .accel_mode = ICM42688_MODE_LOW_NOISE, // 低噪声模式
        .accel_fs = ICM42688_ACCEL_FS_8G,      // ±8g
        .accel_odr = ICM42688_ODR_1000HZ,      // 1000 Hz ODR

        .fifo_enable = false, // 禁用FIFO

        .interrupt = {
            .data_ready_en = true,                    // 使能数据就绪中断
            .int1_mode = ICM42688_INT_PULSE,          // 脉冲模式
            .int1_drive = ICM42688_INT_PUSH_PULL,     // 推挽输出
            .int1_polarity = ICM42688_INT_ACTIVE_HIGH // 高电平有效
        }};

    ret = icm42688_config_sensor(&sensor, &config);
    if (ret != ICM42688_OK)
    {
        printf("传感器配置失败: %s\n", icm42688_get_error_string(ret));
        return -1;
    }

    // 3. 读取传感器数据
    icm42688_sensor_data_t data;
    while (1)
    {
        ret = icm42688_read_sensor_data(&sensor, &data);
        if (ret == ICM42688_OK)
        {
            printf("加速度(m/s²): X=%.2f, Y=%.2f, Z=%.2f\n",
                   data.accel_x, data.accel_y, data.accel_z);
            printf("角速度(rad/s): X=%.2f, Y=%.2f, Z=%.2f\n",
                   data.gyro_x, data.gyro_y, data.gyro_z);
            printf("温度(°C): %.2f\n", data.temperature);
        }

        BSP_Delay(100); // 每100ms读取一次
    }
}