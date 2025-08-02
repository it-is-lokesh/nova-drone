#include <iostream>
#include <nova_processing/nv.hpp>

nv_mat<float64_t, 3, 3> inv_T_matrix(float64_t roll, float64_t pitch){
    nv_mat<float64_t, 3, 3> ret;
    float64_t sin_phi, cos_phi, tan_theta, cos_theta;
    sin_phi = sin(roll);
    cos_phi = cos(roll);
    tan_theta = tan(pitch);
    cos_theta = cos(pitch);

    ret[0][0] = 1;
    ret[0][1] = sin_phi * tan_theta;
    ret[0][2] = cos_phi * tan_theta;
    ret[1][1] = cos_phi;
    ret[1][2] = -sin_phi;
    ret[2][1] = sin_phi/cos_theta;
    ret[2][2] = cos_phi/cos_theta;

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
    nv_mat<float64_t, 3, 1> dorient_w;
    nv_mat<float64_t, 3, 1> orient_w;    

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

        // printf("%f %f %f \n", omega_b[0][0], omega_b[1][0], omega_b[2][0]);

        nv_mat<float64_t, 3, 3> inv_T = inv_T_matrix(omega_b[0][0], omega_b[1][0]);

        nvMatrixMultiply<float64_t, float64_t, float64_t, 3, 3, 1>(inv_T, omega_b, dorient_w);

        orient_w[0][0] = orient_w[0][0] + 0.01 * dorient_w[0][0];
        orient_w[1][0] = orient_w[1][0] + 0.01 * dorient_w[1][0];
        orient_w[2][0] = orient_w[2][0] + 0.01 * dorient_w[2][0];

        printf("%f %f %f \n", orient_w[0][0], orient_w[1][0], orient_w[2][0]);

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