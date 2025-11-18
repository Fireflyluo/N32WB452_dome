#ifndef __IMU_DRIVER_H__
#define __IMU_DRIVER_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "sensors\icm42688p\icm42688_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

    int imu_init1(void);

#ifdef __cplusplus
}
#endif

#endif /* __ICM42688_DRIVER_H__ */