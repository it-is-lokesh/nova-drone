#ifndef _NV_ENUMS_HPP_
#define _NV_ENUMS_HPP_

#include <stdint.h>

typedef int32_t nv_enum;

typedef enum {
    SHM_SENSOR_IMU = 1,
    SHM_SENSOR_ALTIMETER = 2,
    SHM_ACTUATOR_MOTOR_SPEED = 3,
} nv_shm_e;

typedef enum {
    NV_SUCCESS = 0,
    NV_ERROR = -1,
    NV_ERROR_INVALID_PARAM = -2,
    NV_ERROR_SHM = -3,
    NV_ERROR_SEM = -4,
    NV_ERROR_MUTEX = -5,
    NV_ERROR_THREAD = -6,
    NV_ERROR_NOT_IMPLEMENTED = -7,
} nv_status_e;

typedef nv_status_e nv_status;

typedef enum {
    NV_IDX_X = 0,
    NV_IDX_Y = 1,
    NV_IDX_Z = 2,
} nv_axis_e;

#endif /* _NV_ENUMS_HPP_ */