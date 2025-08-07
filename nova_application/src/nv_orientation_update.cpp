#include <iostream>
#include <nova_processing/nv.hpp>


void normalize(double v[3]) {
    double norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    for (int i = 0; i < 3; ++i) v[i] /= norm;
}

void project_out(double target[3], const double ref[3]) {
    double dot = target[0]*ref[0] + target[1]*ref[1] + target[2]*ref[2];
    for (int i = 0; i < 3; ++i) target[i] -= dot * ref[i];
}

void orthonormalize(nv_mat<float64_t, 3, 3>& R) {
    double r1[3] = { R[0][0], R[1][0], R[2][0] };
    double r2[3] = { R[0][1], R[1][1], R[2][1] };

    normalize(r1);

    project_out(r2, r1);
    normalize(r2);

    double r3[3];
    // r3 = r1 x r2
    r3[0] = r1[1]*r2[2] - r1[2]*r2[1];
    r3[1] = r1[2]*r2[0] - r1[0]*r2[2];
    r3[2] = r1[0]*r2[1] - r1[1]*r2[0];
    normalize(r3);

    // Write back into R
    for (int i = 0; i < 3; ++i) {
        R[i][0] = r1[i];
        R[i][1] = r2[i];
        R[i][2] = r3[i];
    }
}

nv_mat<float64_t, 3, 1> get_orient_w(nv_mat<float64_t, 3, 3> &R){
    nv_mat<float64_t, 3, 1> ret;
    ret[0][0] = atan2(R[2][1], R[2][2]);
    if(R[2][0]<=-1)ret[1][0] = -asin(-1);
    else if(R[2][0]>=1)ret[1][0] = -asin(1);
    else ret[1][0] = -asin(R[2][0]);
    ret[2][0] = atan2(R[1][0], R[0][0]);
    return ret;
}

inline nv_mat<float64_t, 3, 3> get_skew_sym_mat(nv_mat<float64_t, 3, 1> &mat){
    nv_mat<float64_t, 3, 3> ret;
    ret[0][1] = -mat[2][0];
    ret[0][2] = mat[1][0];
    ret[1][0] = mat[2][0];
    ret[1][2] = -mat[0][0];
    ret[2][0] = -mat[1][0];
    ret[2][1] = mat[0][0];
    return ret;
}

int main(){

    nv_shm_metadata_t metadata;
    nv_shm_mgr_header header;

    nv_imu_data imu_data;

    int loop;
    int shm_fd;
    int count;

    nv_mat<float64_t, 3, 1> omega_b;
    nv_mat<float64_t, 3, 1> orient_w;
    nv_mat<float64_t, 3, 3> omega_skew;
    nv_mat<float64_t, 3, 3> R;
    R[0][0] = 1;
    R[1][1] = 1;
    R[2][2] = 1;

    count = 10;

    snprintf(metadata.shm_name, SHM_NAME_MAX, "imu_shm\n");
    snprintf(metadata.pname, PROCESS_NAME_MAX, "orient_update\n");

    shm_fd = nvShmManager::nvMapShm(&header);

    nvShmManager::nvSetShmPtr(header, &metadata);

    imu_data = (nv_imu_data)metadata.data->data_ptr;

    loop = 0;

    while(1){
#ifndef USE_SEMAPHORE

#else
        sem_wait(&metadata.data->read_sem);
#endif
        omega_b[0][0] = imu_data[loop%count].angular_velocity.x;
        omega_b[1][0] = imu_data[loop%count].angular_velocity.y;
        omega_b[2][0] = imu_data[loop%count].angular_velocity.z;

        omega_skew = get_skew_sym_mat(omega_b);
        for(int j=0;j<3;j++){
            for(int k=0;k<3;k++){
                omega_skew[j][k] *= 0.01; 
            }
        }
        omega_skew[0][0] += 1;
        omega_skew[1][1] += 1;
        omega_skew[2][2] += 1;

        nvMatrixMultiply<float64_t, float64_t, float64_t, 3, 3, 3>(R, omega_skew, R);

        orient_w = get_orient_w(R);
        printf("loop: %d: %f %f %f \n", loop, orient_w[0][0], orient_w[1][0], orient_w[2][0]);

        if(loop%10 == 0){
            orthonormalize(R);
        }

        loop++;

#ifndef USE_SEMAPHORE

#else
        sem_post(&metadata.data->write_sem);
#endif
    }

    munmap(header, sizeof(nv_shm_mgr_header_t));
    close(shm_fd);

    return 0;
}