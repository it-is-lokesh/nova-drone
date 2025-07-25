#include <iostream>
using namespace std;

#include <nova_processing/nv.hpp>

int main(){
    nv_mat<int32_t, 2, 2> mat1;
    nv_mat<int32_t, 2, 2> mat2;
    nv_mat<int32_t, 2, 2> mat3;

    mat1[0][0] = 1;
    mat1[0][1] = 2;
    mat1[1][0] = 3;
    mat1[1][1] = 4;

    mat2[0][0] = 1;
    mat2[0][1] = 2;
    mat2[1][0] = 3;
    mat2[1][1] = 4;

    mat3[0][0] = 1;
    mat3[0][1] = 2;
    mat3[1][0] = 3;
    mat3[1][1] = 4;

    nvMatrixMultiply<int32_t, int32_t, int32_t, 2, 2, 2>(mat1, mat2, mat3);

    printf("vals: %d %d %d %d \n", mat3[0][0], mat3[0][1], mat3[1][0], mat3[1][1]);

    return 0;
}