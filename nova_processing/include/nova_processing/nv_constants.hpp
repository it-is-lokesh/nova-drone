#ifndef __NV_CONSTANTS_HPP__
#define __NV_CONSTANTS_HPP__

#include <nova_processing/nv.hpp>

struct nvConstants {
    inline static float64_t nv_g = 9.8;
    inline static nv_vec<float64_t, 3> nvGravity = {0.0, 0.0, -nv_g};
};


#endif // __NV_CONSTANTS_HPP__