#ifndef _NV_MATH_HPP_
#define _NV_MATH_HPP_

#include <math.h>
#include <stdint.h>
#include <pthread.h>

#include <nova_processing/nv_types.hpp>

template<typename T, uint32_t P, uint32_t Q>
struct nv_mat {
    T data[P][Q] = {};

    T* operator[](size_t i) { return data[i]; }
    const T* operator[](size_t i) const { return data[i]; }
};

template<typename T, uint32_t N>
struct nv_vec {
    nv_mat<T, N, 1> mat;

    // Indexing (one bracket only)
    T& operator[](size_t i) {
        return mat[i][0]; // access row i, column 0
    }

    const T& operator[](size_t i) const {
        return mat[i][0];
    }
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

// Matrix × Vector = Vector
template<typename In1, typename In2, typename Out, uint32_t P, uint32_t Q>
void nvMatrixMultiply(
    nv_mat<In1, P, Q>& A,
    nv_vec<In2, Q>& B,
    nv_vec<Out, P>& C
) {
    for (uint32_t i = 0; i < P; i++) {
        Out sum = 0;
        for (uint32_t k = 0; k < Q; k++) {
            sum += static_cast<Out>(A[i][k]) * static_cast<Out>(B[k]);
        }
        C[i] = sum;
    }
}

// Vector × Matrix = Vector
template<typename In1, typename In2, typename Out, uint32_t Q, uint32_t R>
void nvMatrixMultiply(
    nv_vec<In1, Q>& A,
    nv_mat<In2, Q, R>& B,
    nv_vec<Out, R>& C
) {
    for (uint32_t j = 0; j < R; j++) {
        Out sum = 0;
        for (uint32_t k = 0; k < Q; k++) {
            sum += static_cast<Out>(A[k]) * static_cast<Out>(B[k][j]);
        }
        C[j] = sum;
    }
}

void nvNormalizeVector(double v[3]);

void nvProjectOut(double target[3], const double ref[3]);

void nvOrthonormalizeMatrix(nv_mat<float64_t, 3, 3>& R);

nv_vec<float64_t, 3> nvGetOrientationFromRotMat(nv_mat<float64_t, 3, 3> &R);

inline nv_mat<float64_t, 3, 3> nvGetSkewSymmetricMatrix(nv_vec<float64_t, 3> &vec){
    nv_mat<float64_t, 3, 3> ret;
    ret[0][1] = -vec[2];
    ret[0][2] = vec[1];
    ret[1][0] = vec[2];
    ret[1][2] = -vec[0];
    ret[2][0] = -vec[1];
    ret[2][1] = vec[0];
    return ret;
}

template<typename T, uint32_t P, uint32_t Q>
void nvInitializeIdentityMatrix(nv_mat<T, P, Q> &matrix) {
    for (uint32_t i = 0; i < P; i++) {
        for (uint32_t j = 0; j < Q; j++) {
            if (i == j) {
                matrix[i][j] = 1;
            } else {
                matrix[i][j] = 0;
            }
        }
    }
}

#endif /* _NV_MATH_HPP_ */