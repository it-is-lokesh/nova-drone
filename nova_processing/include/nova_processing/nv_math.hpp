#ifndef _NV_MATH_HPP_
#define _NV_MATH_HPP_

#include <math.h>
#include <stdint.h>
#include <pthread.h>
#include <stdio.h>

#include <nova_processing/nv_types.hpp>

template<typename T, uint32_t P, uint32_t Q>
struct nv_mat;

template<typename T, uint32_t P>
struct nv_vec {
    T data[P] = {};
    uint32_t shape = {P};
    T* begin = &data[0];


    T& operator[](uint32_t i){ return data[i]; }
    const T& operator[](uint32_t i) const { return data[i]; }

    T operator*(const nv_vec<T, P> &vec2) {
        T ret = T(0);
        for(uint32_t i=0; i<P; i++){
            ret += data[i] * vec2[i];
        }
        return ret;
    }

    template<typename U>
    nv_vec<T, P> operator*(const U &scalar) const {
        nv_vec<T, P> result;
        for (uint32_t i = 0; i < P; i++) {
            result[i] = data[i] * scalar;
        }
        return result;
    }

    nv_vec<T, P> operator+(const nv_vec<T, P> &vec2) const {
        nv_vec<T, P> result;
        for (uint32_t i = 0; i < P; i++) {
            result[i] = data[i] + vec2[i];
        }
        return result;
    }

    nv_vec<T, P> operator-(const nv_vec<T, P> &vec2) const {
        nv_vec<T, P> result;
        for (uint32_t i = 0; i < P; i++) {
            result[i] = data[i] - vec2[i];
        }
        return result;
    }

    void operator=(const T* vec2) {
        for(uint32_t i=0; i<P; i++){
            data[i] = vec2[i];
        }
    }

    nv_mat<T, 3, 3> get_skew_sym_mat() {
        nv_mat<T, 3, 3> ret;
        ret[0][1] = -data[2];
        ret[0][2] = data[1];
        ret[1][0] = data[2];
        ret[1][2] = -data[0];
        ret[2][0] = -data[1];
        ret[2][1] = data[0];
        return ret;
    }
};

template<typename T, uint32_t P, uint32_t Q>
struct nv_mat {
    T data[P][Q] = {};
    uint32_t shape[2] = {P, Q};
    T* begin = &data[0][0];

    nv_mat<T, Q, P> get_transpose() {
        nv_mat<T, Q, P> ret;
        for(uint32_t i=0; i<P; i++){
            for(uint32_t j=0; j<Q; j++){
                ret[j][i] = data[i][j];
            }
        }
        return ret;
    }

    nv_mat<T, P, Q> get_inverse() {
        nv_mat<T, P, Q> aug = *this;

        // Initialize inverse as identity matrix
        nv_mat<T, P, Q> inverse;
        for (uint32_t i = 0; i < P; i++) {
            for (uint32_t j = 0; j < Q; j++) {
                inverse[i][j] = (i == j) ? T(1) : T(0);
            }
        }

        const T EPS = 1e-9;

        for (uint32_t i = 0; i < P; i++) {
            // Pivot: find row with largest element in column i
            uint32_t pivot = i;
            for (uint32_t j = i + 1; j < P; j++) {
                if (std::fabs(aug[j][i]) > std::fabs(aug[pivot][i])) {
                    pivot = j;
                }
            }

            if (std::fabs(aug[pivot][i]) < EPS) {
                return inverse; // singular
            }

            // Swap rows in aug and inverse
            if (pivot != i) {
                for (uint32_t k = 0; k < Q; k++) {
                    std::swap(aug[i][k], aug[pivot][k]);
                    std::swap(inverse[i][k], inverse[pivot][k]);
                }
            }

            // Normalize pivot row
            T div = aug[i][i];
            for (uint32_t k = 0; k < Q; k++) {
                aug[i][k] /= div;
                inverse[i][k] /= div;
            }

            // Eliminate other rows
            for (uint32_t j = 0; j < P; j++) {
                if (j == i) continue;
                T factor = aug[j][i];
                for (uint32_t k = 0; k < Q; k++) {
                    aug[j][k] -= factor * aug[i][k];
                    inverse[j][k] -= factor * inverse[i][k];
                }
            }
        }

        return inverse;
    }

    T* operator[](uint32_t i){ return data[i]; }
    const T* operator[](uint32_t i) const { return data[i]; }
    
    template<uint32_t R>
    nv_mat<T, P, R> operator*(const nv_mat<T, Q, R> &mat2) const {
        nv_mat<T, P, R> ret;
        for(uint32_t i=0; i<P; i++){
            for(uint32_t j=0; j<R; j++){
                // int sum = 0;
                for(uint32_t k=0; k<Q; k++){
                    ret[i][j] += data[i][k] * mat2[k][j];
                }
            }
        }
        return ret;
    }

    nv_vec<T, P> operator*(const nv_vec<T, Q> &vec2){
        nv_vec<T, P> ret;
        for(uint32_t i=0; i<P; i++){
            for(uint32_t j=0; j<Q; j++){
                ret[i] += data[i][j] * vec2[j];
            }
        }
        return ret;
    }

    template<typename U>
    nv_mat<T, P, Q> operator*(U scalar) const {
        nv_mat<T, P, Q> result;
        for (uint32_t i = 0; i < P; i++) {
            for (uint32_t j = 0; j < Q; j++) {          
                result[i][j] = data[i][j] * scalar;
            }
        }
        return result;
    }

    nv_mat<T, P, Q> operator+(const nv_mat<T, P, Q> &mat2) const {
        nv_mat<T, P, Q> result;
        for (uint32_t i = 0; i < P; i++) {
            for (uint32_t j = 0; j < Q; j++) {          
                result[i][j] = data[i][j] + mat2[i][j];
            }
        }
        return result;
    }

    nv_mat<T, P, Q> operator-(const nv_mat<T, P, Q> &mat2) const {
        nv_mat<T, P, Q> result;
        for (uint32_t i = 0; i < P; i++) {
            for (uint32_t j = 0; j < Q; j++) {          
                result[i][j] = data[i][j] - mat2[i][j];
            }
        }
        return result;
    }

    void operator=(const T* mat2) {
        for(uint32_t i=0; i<P*Q; i++){
            data[i/Q][i%Q] = mat2[i];
        }
    }
};

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
nv_status nvInitializeIdentityMatrix(nv_mat<T, P, Q> &matrix) {
    for (uint32_t i = 0; i < P; i++) {
        for (uint32_t j = 0; j < Q; j++) {
            if (i == j) {
                matrix[i][j] = 1;
            } else {
                matrix[i][j] = 0;
            }
        }
    }
    return NV_SUCCESS;
}

template<typename T, uint32_t P, uint32_t Q>
nv_mat<T, Q, P> nvTransposeMatrix(nv_mat<T, P, Q> &matrix) {
    nv_mat<T, Q, P> ret;
    for (uint32_t i = 0; i < P; i++) {
        for (uint32_t j = 0; j < Q; j++) {
            ret[j][i] = matrix[i][j];
        }
    }
    return ret;
}

template<typename T, uint32_t P, uint32_t Q, uint32_t R, uint32_t S>
nv_status nvSetMatrixUsingSlices(nv_mat<T, P, Q> &dst, nv_mat<T, R, S> &src, nv_vec<uint32_t, 2> start) {
    for(uint32_t i=0;i<R;i++){
        for(uint32_t j=0;j<S;j++){
            dst[i+start[0]][j+start[1]] = src[i][j];
        }
    }

    return NV_SUCCESS;
}

nv_status nvGetEulerDot(nv_vec<float64_t,3>& omega, nv_vec<float64_t,3>& ang, nv_vec<float64_t,3>& out);

nv_status nvEulerToRot(nv_vec<float64_t,3>& ang, nv_mat<float64_t,3,3>& R);

#endif /* _NV_MATH_HPP_ */