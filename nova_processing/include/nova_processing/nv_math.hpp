#ifndef _NV_MATH_HPP_
#define _NV_MATH_HPP_

#include <math.h>

template<typename T, uint32_t P, uint32_t Q>
struct nv_mat {
    T data[P][Q] = {};

    T* operator[](size_t i) { return data[i]; }
    const T* operator[](size_t i) const { return data[i]; }
};

template<
    typename In1, typename In2, typename Out,
    uint32_t P, uint32_t Q, uint32_t R
>
void nvMatrixMultiply(
    nv_mat<In1, P, Q> &A, 
    nv_mat<In2, Q, R> &B, 
    nv_mat<Out, P, R> &C
){
    uint32_t i, j, k;
    Out sum;

    for(i = 0; i < P; i++){
        for(j = 0; j < R; j++){
            sum = 0;
            for(k = 0; k < Q; k++){
                sum += (Out) A[i][k] * (Out) B[k][j];
            }
            C[i][j] = sum;
        }
    }
}

#endif /* _NV_MATH_HPP_ */