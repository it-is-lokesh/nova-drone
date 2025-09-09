#include <iostream>
#include <nova_processing/nv.hpp>


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

        omega_skew = nvGetSkewSymmetricMatrix(omega_b);
        for(int j=0;j<3;j++){
            for(int k=0;k<3;k++){
                omega_skew[j][k] *= 0.01; 
            }
        }
        omega_skew[0][0] += 1;
        omega_skew[1][1] += 1;
        omega_skew[2][2] += 1;

        nvMatrixMultiply<float64_t, float64_t, float64_t, 3, 3, 3>(R, omega_skew, R);

        orient_w = nvGetOrientationFromRotMat(R);
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