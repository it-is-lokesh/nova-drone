#ifndef _NV_TYPES_HPP_
#define _NV_TYPES_HPP_

#include <stdint.h>
#include <nova_processing/nv_enums.hpp>
#include <semaphore.h>
#include <pthread.h>
#include <mutex>
#include <thread>
#include <chrono>

#define COVARIANCE_MAX (9)
#define USE_SEMAPHORE

typedef float float32_t;
typedef double float64_t;
typedef bool nv_bool;
typedef size_t nv_size;
typedef nv_enum nv_status;

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

typedef _nv_reference* nv_reference;

typedef _nv_orientation* nv_orientation;

typedef _nv_angular_velocity* nv_angular_velocity;

typedef _nv_linear_acceleration* nv_linear_acceleration;

typedef _nv_imu_data* nv_imu_data;

typedef _nv_altimeter_data* nv_altimeter_data;

// typedef _nv_shm_obj* nv_shm_obj;

class nvPingPongCounter {
    int32_t value;
    int32_t max;

public:
    explicit nvPingPongCounter(int32_t start = 0, int32_t max_val = 0)
        : value(start), max(max_val) {}

    // inline getters
    inline int32_t get() const { return value; }
    inline int32_t next() { value = (value + 1) % (max + 1); return value; }

    // assignment
    inline nvPingPongCounter& operator=(int32_t v) {
        value = v % (max + 1);
        return *this;
    }

    // prefix increment/decrement
    inline nvPingPongCounter& operator++() { next(); return *this; }
    inline nvPingPongCounter& operator--() {
        value = (value - 1 + (max + 1)) % (max + 1);
        return *this;
    }

    // postfix increment/decrement
    inline int32_t operator++(int32_t) { int32_t old = value; ++(*this); return old; }
    inline int32_t operator--(int32_t) { int32_t old = value; --(*this); return old; }

    // arithmetic
    inline nvPingPongCounter& operator*=(int32_t factor) {
        value = (value * factor) % (max + 1);
        return *this;
    }
    inline nvPingPongCounter& operator/=(int32_t divisor) {
        value = (divisor ? value / divisor : 0) % (max + 1);
        return *this;
    }

    // implicit conversion to int32_t
    inline operator int32_t() const { return value; }
};

template<typename T>
class nvSecureVariable {
    T value;
    mutable std::mutex mtx;

public:
    nvSecureVariable() = default;
    nvSecureVariable(const T& v) : value(v) {}

    // Assignment
    nvSecureVariable& operator=(const T& v) {
        std::lock_guard<std::mutex> lock(mtx);
        value = v;
        return *this;
    }

    // Conversion to T (read access)
    operator T() const {
        std::lock_guard<std::mutex> lock(mtx);
        return value;
    }

    // +=
    nvSecureVariable& operator+=(const T& v) {
        std::lock_guard<std::mutex> lock(mtx);
        value += v;
        return *this;
    }

    // -=
    nvSecureVariable& operator-=(const T& v) {
        std::lock_guard<std::mutex> lock(mtx);
        value -= v;
        return *this;
    }

    // *=
    nvSecureVariable& operator*=(const T& v) {
        std::lock_guard<std::mutex> lock(mtx);
        value *= v;
        return *this;
    }

    // /=
    nvSecureVariable& operator/=(const T& v) {
        std::lock_guard<std::mutex> lock(mtx);
        value /= v;
        return *this;
    }

    // Prefix ++
    nvSecureVariable& operator++() {
        std::lock_guard<std::mutex> lock(mtx);
        ++value;
        return *this;
    }

    // Postfix ++
    nvSecureVariable operator++(int) {
        std::lock_guard<std::mutex> lock(mtx);
        nvSecureVariable tmp = *this;
        value++;
        return tmp;
    }

    // Prefix --
    nvSecureVariable& operator--() {
        std::lock_guard<std::mutex> lock(mtx);
        --value;
        return *this;
    }

    // Postfix --
    nvSecureVariable operator--(int) {
        std::lock_guard<std::mutex> lock(mtx);
        nvSecureVariable tmp = *this;
        value--;
        return tmp;
    }

    template<typename F>
    auto with_lock(F&& f) -> decltype(f(value)) {
        std::lock_guard<std::mutex> lock(mtx);
        return f(value);
    }
};



#endif /* _NV_TYPES_HPP_ */