#ifndef __SENSOR_COMMON_H__
#define __SENSOR_COMMON_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C"
{
#endif
    /* 传感器数据类型 */
    typedef struct
    {
        float accel[3]; // 加速度 (m/s2)
        float gyro[3];  // 角速度 (rad/s)
        float temp;     // 温度 (°C)
    } sensor_data_t;

    /* 统一传感器接口结构 */
    typedef struct
    {
        int32_t (*init)(void);    // 初始化设备
        int32_t (*read_data)(sensor_data_t *data); // 读取传感器数据
        int32_t (*control)(int32_t cmd, void *arg);  // 控制命令接口
        int32_t (*sleep)(void);                    // 进入低功耗模式
        int32_t (*wakeup)(void);                   // 唤醒设备
    } sensor_device_t;

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_COMMON_H__ */