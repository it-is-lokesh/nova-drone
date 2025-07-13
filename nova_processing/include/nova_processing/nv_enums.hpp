#ifndef _NV_ENUMS_HPP_
#define _NV_ENUMS_HPP_

#include <stdint.h>

typedef int32_t nv_enum;

typedef enum {
    SHM_SENSOR_IMU = 1,
    SHM_SENSOR_ALTIMETER = 2,
    SHM_ACTUATOR_MOTOR_SPEED = 3,
} nv_shm_e;

#endif /* _NV_ENUMS_HPP_ */