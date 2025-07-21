#ifndef _NV_TYPES_HPP_
#define _NV_TYPES_HPP_

#include <stdint.h>
#include <nova_processing/nv_enums.hpp>
#include <semaphore.h>
#include <pthread.h>

#define COVARIANCE_MAX (9)
#define SHM_NAME_MAX (10)
#define USE_SEMAPHORE

typedef float float32_t;
typedef double float64_t;
typedef bool nv_bool;
typedef int32_t nv_size;

typedef struct nv_reference_t {
    
} _nv_reference;

typedef struct nv_orientation_t {
    nv_reference_t base;

    float64_t x;
    float64_t y;
    float64_t z;
    float64_t w;
    float64_t covariance[COVARIANCE_MAX];
} _nv_orientation;

typedef struct nv_angular_velocity_t {
    nv_reference_t base;

    float64_t x;
    float64_t y;
    float64_t z;
    float64_t covariance[COVARIANCE_MAX];
} _nv_angular_velocity;

typedef struct nv_linear_acceleration_t {
    nv_reference_t base;

    float64_t x;
    float64_t y;
    float64_t z;
    float64_t covariance[COVARIANCE_MAX];
} _nv_linear_acceleration;

typedef struct nv_imu_data_t {
    nv_reference_t base;

    int32_t sec;
    uint32_t nsec;
    nv_linear_acceleration_t linear_acceleration;
    nv_angular_velocity_t angular_velocity;
    nv_orientation_t orientation;
} _nv_imu_data;

typedef struct nv_altimeter_data_t {
    nv_reference_t base;

    float32_t vertical_position;
    float32_t vertical_reference;
    float32_t vertical_velocity;
} _nv_altimeter_data;

typedef struct nv_shm_data_t {
    void *data_ptr;
#ifndef USE_SEMAPHORE
    pthread_mutex_t lock;
    pthread_mutexattr_t attr;
    pthread_cond_t cond;
    nv_bool flag;
#else
    sem_t write_sem;
    sem_t read_sem;
} _nv_shm_data;

typedef _nv_shm_data* nv_shm_data;

typedef struct nv_shm_obj_t {
    nv_reference_t base;

    nv_enum shm_id;
    char shm_name[SHM_NAME_MAX];
    nv_size shm_size;
    nv_shm_data data;
#endif
} _nv_shm_obj;


typedef _nv_reference* nv_reference;

typedef _nv_orientation* nv_orientation;

typedef _nv_angular_velocity* nv_angular_velocity;

typedef _nv_linear_acceleration* nv_linear_acceleration;

typedef _nv_imu_data* nv_imu_data;

typedef _nv_altimeter_data* nv_altimeter_data;

typedef _nv_shm_data* nv_shm_data;

typedef _nv_shm_obj* nv_shm_obj;




#endif /* _NV_TYPES_HPP_ */