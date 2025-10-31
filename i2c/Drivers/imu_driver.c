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
        .gyro_odr = ICM42688_ODR_100HZ,      // 1000 Hz ODR

        .accel_mode = ICM42688_MODE_LOW_NOISE, // 低噪声模式
        .accel_fs = ICM42688_ACCEL_FS_2G,      // ±2g
        .accel_odr = ICM42688_ODR_100HZ,      // 100 Hz ODR


      };

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
            printf("加速度(m/s²): X=%.2f, Y=%.2f, Z=%.2f\r\n",
                   data.accel_x, data.accel_y, data.accel_z);
            printf("角速度(rad/s): X=%.2f, Y=%.2f, Z=%.2f\r\n",
                   data.gyro_x, data.gyro_y, data.gyro_z);
            printf("温度(°C): %.2f\r\n", data.temperature);
        }

        BSP_Delay(100); // 每100ms读取一次
    }
}
typedef struct
{
    int16_t x; /**< Raw int16_t value from the x axis */
    int16_t y; /**< Raw int16_t value from the y axis */
    int16_t z; /**< Raw int16_t value from the z axis */
} icm42688RawData_t;

typedef struct
{
    float x; /**< value from the x axis */
    float y; /**< value from the y axis */
    float z; /**< value from the z axis */
} icm42688RealData_t;
void imu_init2(void)
{
    uint8_t reg_val = 0;
    /* 读取 who am i 寄存器 */
    i2c_read_reg(0x69, ICM42688_REG_WHO_AM_I, &reg_val, 1);

    icm42688_reg_bank_sel_t bank_sel = {.bits.USER_BANK = 0};
    i2c_write_reg(0x69, ICM42688_REG_BANK_SEL, &bank_sel.reg, 1); // 设置bank 0区域寄存器

    reg_val = 0x01;
    i2c_write_reg(0x69, ICM42688_REG_DEVICE_CONFIG, &reg_val, 1); // 软复位传感器
    BSP_Delay(500);

    i2c_write_reg(0x69, ICM42688_REG_BANK_SEL, &bank_sel.reg, 1); // 设置bank 0区域寄存器
    reg_val = (0x02 << 5);                                        // 量程 ±2g
    reg_val |= (0x08);                                            // 输出速率 100HZ
    i2c_write_reg(0x69, ICM42688_REG_ACCEL_CONFIG0, &reg_val, 1);

    i2c_write_reg(0x69, ICM42688_REG_BANK_SEL, &bank_sel.reg, 1); // 设置bank 0区域寄存器
    // reg_val = icm42688_read_reg(ICM42688_GYRO_CONFIG0);//page73
    reg_val = (0x01 << 5); // 量程 ±1000dps
    reg_val |= (0x08);     // 输出速率 100HZ
    i2c_write_reg(0x69, ICM42688_REG_GYRO_CONFIG0, &reg_val, 1);

    i2c_write_reg(0x69, ICM42688_REG_BANK_SEL, &bank_sel.reg, 1); // 设置bank 0区域寄存器
    reg_val = 0;
    i2c_read_reg(0x69, ICM42688_REG_PWR_MGMT0, &reg_val, 1); // 读取PWR—MGMT0当前寄存器的值(page72)
    reg_val &= ~(1 << 5);                                    // 使能温度测量
    reg_val |= ((3) << 2);                                   // 设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
    reg_val |= (3);                                          // 设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
    i2c_write_reg(0x69, ICM42688_REG_PWR_MGMT0, &reg_val, 1);
    BSP_Delay(100); // 操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作

    while (1)
    {
        uint8_t buffer[12] = {0};
        icm42688RawData_t accRaw;
        icm42688RawData_t gyroRaw;

        i2c_read_reg(0x69, ICM42688_REG_ACCEL_DATA_X1, buffer, 12);

        accRaw.x = ((uint16_t)buffer[0] << 8) | buffer[1];
        accRaw.y = ((uint16_t)buffer[2] << 8) | buffer[3];
        accRaw.z = ((uint16_t)buffer[4] << 8) | buffer[5];
        gyroRaw.x = ((uint16_t)buffer[6] << 8) | buffer[7];
        gyroRaw.y = ((uint16_t)buffer[8] << 8) | buffer[9];
        gyroRaw.z = ((uint16_t)buffer[10] << 8) | buffer[11];
        BSP_Delay(100);
    }
}
