#include "icm42688_hal.h"
#include "icm42688p_reg.h"
#include <rtdevice.h>

static struct rt_i2c_bus_device *i2c_bus = RT_NULL;

/* I2C写寄存器函数 */
icm42688_err_t icm42688_i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr,
                                      const uint8_t *data, uint16_t len)
{
    struct rt_i2c_msg msgs[2];
    uint8_t *write_buf;

    if (i2c_bus == RT_NULL)
    {
        return ICM42688_ERR_COMM;
    }

    /* 分配缓冲区：寄存器地址 + 数据 */
    write_buf = rt_malloc(len + 1);
    if (write_buf == RT_NULL)
    {
        return ICM42688_ERR_COMM;
    }

    write_buf[0] = reg_addr;
    rt_memcpy(&write_buf[1], data, len);

    msgs[0].addr = dev_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = write_buf;
    msgs[0].len = len + 1;

    if (rt_i2c_transfer(i2c_bus, msgs, 1) == 1)
    {
        rt_free(write_buf);
        return ICM42688_OK;
    }

    rt_free(write_buf);
    return ICM42688_ERR_COMM;
}

/* I2C读寄存器函数 */
icm42688_err_t icm42688_i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr,
                                     uint8_t *data, uint16_t len)
{
    struct rt_i2c_msg msgs[2];

    if (i2c_bus == RT_NULL)
    {
        return ICM42688_ERR_COMM;
    }

    /* 先写寄存器地址 */
    msgs[0].addr = dev_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg_addr;
    msgs[0].len = 1;

    /* 然后读取数据 */
    msgs[1].addr = dev_addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = data;
    msgs[1].len = len;

    if (rt_i2c_transfer(i2c_bus, msgs, 2) == 2)
    {
        return ICM42688_OK;
    }

    return ICM42688_ERR_COMM;
}
/* 延时函数 */
void icm42688_delay_ms(uint32_t ms)
{
    rt_thread_mdelay(ms);
}

/* 获取tick函数（可选） */
uint32_t icm42688_get_tick(void)
{
    return rt_tick_get();
}

icm42688_hal_handle_t imu_handle;

int imu_init1(void)
{
    icm42688_comm_config_t comm_config;

    /* 查找I2C总线设备 */
    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find("i2c1");
    if (i2c_bus == RT_NULL)
    {
        rt_kprintf("I2C bus not found!\n");
        return -RT_ERROR;
    }

    /* 配置通信接口 */
    comm_config.interfaces = ICM42688_INTERFACE_I2C;
    comm_config.dev_addr = ICM42688P_I2C_ADDR_0; // 根据硬件连接选择地址
    comm_config.write_reg = icm42688_i2c_write_reg;
    comm_config.read_reg = icm42688_i2c_read_reg;
    comm_config.delay_ms = icm42688_delay_ms;
    comm_config.get_tick = icm42688_get_tick;
    comm_config.lock = RT_NULL; // 如果不需要线程安全可以设为NULL
    comm_config.unlock = RT_NULL;

    /* 初始化设备 */
    icm42688_err_t ret = icm42688_hal_init(&imu_handle, &comm_config);
    if (ret != ICM42688_OK)
    {
        rt_kprintf("ICM42688 init failed: %d\n", ret);
        return -RT_ERROR;
    }

    /* 配置陀螺仪和加速度计 */
    icm42688_hal_set_gyro_config(&imu_handle, ICM42688_GYRO_FS_250DPS, ICM42688_ODR_1000HZ);
    icm42688_hal_set_accel_config(&imu_handle, ICM42688_ACCEL_FS_2G, ICM42688_ODR_1000HZ);

    /* 电源管理 */
    icm42688_hal_config_power_mode(&imu_handle, ICM42688_MODE_LOW_NOISE, ICM42688_MODE_LOW_NOISE);

    rt_kprintf("ICM42688 initialized successfully!\n");
    return RT_EOK;
}
INIT_APP_EXPORT(imu_init1); // 自动初始化
