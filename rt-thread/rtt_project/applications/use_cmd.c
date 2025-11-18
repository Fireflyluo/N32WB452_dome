/*
* 自定义的 msh 命令，可以在 msh 模式下被运行，将一个命令导出到 msh 模式可以使用如下宏接口：
* MSH_CMD_EXPORT(cmd, desc)；
* cmd：命令函数名称
* desc：命令描述字符串
* 导出有参数的命令时，函数的入参为 int argc 和 char**argv。
* argc：命令参数个数
* argv：命令参数字符串指针数组
*
*/
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

/**
 * 自定义的 I2C 扫描命令
 *
 * 命令格式：i2c_scan [bus_name]
 * bus_name：可选参数，指定要扫描的 I2C 总线名称，默认为 "i2c1"
 *
 * 扫描指定总线上的 I2C 设备，并打印找到的设备地址。
 *
 * @param argc 命令参数个数
 * @param argv 命令参数数组

*/
static void i2c_scan(int argc, char *argv[])
{
    struct rt_i2c_bus_device *bus;
    const char *bus_name;

    // 处理参数
    if (argc == 2)
    {
        bus_name = argv[1];
    }
    else
    {
        bus_name = "i2c1"; // 默认总线
    }

    rt_kprintf("Scanning I2C bus: %s\n", bus_name);

    // 1. 查找 I2C 总线设备
    bus = (struct rt_i2c_bus_device *)rt_device_find(bus_name);
    if (bus == RT_NULL)
    {
        rt_kprintf("Error: I2C bus '%s' not found!\n", bus_name);
        return;
    }

    // 2. 验证设备类型（重要！避免断言失败）
    if (bus->parent.type != RT_Device_Class_I2CBUS)
    {
        rt_kprintf("Error: '%s' is not an I2C bus device! Type: %d\n",
                   bus_name, bus->parent.type);
        return;
    }

    rt_kprintf("I2C bus found, starting scan...\n");

    // 3. 扫描地址范围
    int found_count = 0;

    for (rt_uint8_t addr = 1; addr < 125; addr++)
    {
        struct rt_i2c_msg msgs;
        rt_uint8_t dummy;
        rt_err_t result;

        // 配置消息
        msgs.addr = addr;
        msgs.flags = RT_I2C_RD;
        msgs.buf = &dummy;
        msgs.len = 1;

        // 4. 使用带超时的传输（避免阻塞）
        result = rt_i2c_transfer(bus, &msgs, 1);

        if (result == 1)
        {
            rt_kprintf("Device found at 0x%02X\n", addr);
            found_count++;
        }
        else if (result == -RT_ETIMEOUT)
        {
            rt_kprintf("Timeout at address 0x%02X\n", addr);
        }
        // 其他错误不打印，避免输出过多

        // 短暂延时，避免总线过载
        rt_thread_mdelay(1);
    }

    rt_kprintf("Scan completed. Found %d device(s).\n", found_count);
}
MSH_CMD_EXPORT(i2c_scan, "i2c_scan [bus_name] - Scan I2C bus for devices");