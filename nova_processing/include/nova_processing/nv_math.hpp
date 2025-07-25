#ifndef _NV_MATH_HPP_
#define _NV_MATH_HPP_

template<typename T, uint32_t P, uint32_t Q>
using nv_mat = T[P][Q];

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