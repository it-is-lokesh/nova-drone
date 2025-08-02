#include <iostream>
#include <nova_processing/nv.hpp>

nv_mat<float64_t, 3, 1> get_orient_w(nv_mat<float64_t, 3, 3> &R){
    nv_mat<float64_t, 3, 1> ret;
    ret[0][0] = R[2][1]==0?0.0:atan2(R[2][1], R[2][2]);
    ret[1][0] = asin(R[2][0]);
    ret[2][0] = R[1][0]==0?0.0:atan2(R[1][0], R[0][0]);
    return ret;
}

nv_mat<float64_t, 3, 3> get_skew_sym_mat(nv_mat<float64_t, 3, 1> &mat){
    nv_mat<float64_t, 3, 3> ret;
    ret[0][1] = -mat[2][0];
    ret[0][2] = mat[1][0];
    ret[1][0] = mat[2][0];
    ret[1][2] = -mat[0][0];
    ret[2][0] = -mat[1][0];
    ret[2][1] = mat[0][0];
    for(int j=0;j<3;j++){
        for(int k=0;k<3;k++){
            ret[j][k] *= 0.01; 
        }
    }
    ret[0][0] += 1;
    ret[1][1] += 1;
    ret[2][2] += 1;
    return ret;
}

int main(){
    nv_mat<float64_t, 3, 3> R;
    R[0][0] = 1;
    R[1][1] = 1;
    R[2][2] = 1;

    nv_mat<float64_t, 3, 1> orient_w;
    nv_mat<float64_t, 3, 1> omega_b;
    nv_mat<float64_t, 3, 3> omega_skew;
    omega_b[0][0] = 3.14/10;
    omega_b[1][0] = 0;
    omega_b[2][0] = 0;
    omega_skew = get_skew_sym_mat(omega_b);

    for(int j=0;j<3;j++){
        for(int k=0;k<3;k++){
            printf("%f ", omega_skew[j][k]); 
        }
        printf("\n");
    }

    for(int i=0;i<100;i++){
        nvMatrixMultiply<float64_t, float64_t, float64_t, 3, 3, 3>(R, omega_skew, R);
    }
    
    orient_w = get_orient_w(R);
    printf("orientation: %f %f %f \n", orient_w[0][0], orient_w[1][0], orient_w[2][0]);

    omega_b[0][0] = 0;
    omega_b[1][0] = 3.14/4;
    omega_b[2][0] = 0;
    omega_skew = get_skew_sym_mat(omega_b);

    for(int j=0;j<3;j++){
        for(int k=0;k<3;k++){
            printf("%f ", omega_skew[j][k]); 
        }
        printf("\n");
    }

    for(int i=0;i<100;i++){
        nvMatrixMultiply<float64_t, float64_t, float64_t, 3, 3, 3>(R, omega_skew, R);
    }
    
    orient_w = get_orient_w(R);
    printf("orientation: %f %f %f \n", orient_w[0][0], orient_w[1][0], orient_w[2][0]);
    
    return 0;    
}
