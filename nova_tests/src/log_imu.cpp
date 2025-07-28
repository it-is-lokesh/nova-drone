#include <stdio.h>

#include <nova_processing/nv.hpp>


int main(){
    nv_shm_metadata_t metadata;
    nv_shm_mgr_header header;
    nv_imu_data imu_data;

    int loop;
    int shm_fd;

    snprintf(metadata.shm_name, SHM_NAME_MAX, "test_shm_1\n");
    snprintf(metadata.pname, SHM_NAME_MAX, "test_process_1\n");

    shm_fd = nvShmManager::nvMapShm(&header);

    nvShmManager::nvSetShmPtr(header, &metadata);
    
    imu_data = (nv_imu_data)metadata.data->data_ptr;

    loop = 0;

    // Open file to store the data in a csv file
    FILE *fp;

    fp = fopen("imu_logs.csv", "a+");

    fprintf(fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", "sec", "nsec", "lin_acc.x", "lin_acc.y", "lin_acc.z", 
                                            "ang_vel.x", "ang_vel.y", "ang_vel.z",
                                            "orient.x", "orient.y", "orient.z", "orient.w"
                                            );

    while(1){
#ifndef USE_SEMAPHORE

#else
    sem_wait(&metadata.data->read_sem);
#endif

    fprintf(fp, "%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
        imu_data->sec,
        imu_data->nsec,
        imu_data->linear_acceleration.x,
        imu_data->linear_acceleration.y,
        imu_data->linear_acceleration.z,
        imu_data->angular_velocity.x,
        imu_data->angular_velocity.y,
        imu_data->angular_velocity.z,
        imu_data->orientation.x,
        imu_data->orientation.y,
        imu_data->orientation.z,
        imu_data->orientation.w
    );

    // printf("linear acceleration: \n");
    // printf("    x: %f \n", imu_data->linear_acceleration.x);
    // printf("    y: %f \n", imu_data->linear_acceleration.y);
    // printf("    z: %f \n", imu_data->linear_acceleration.z);
    // printf("\n");
    // printf("angular velocity: \n");
    // printf("    x: %f \n", imu_data->angular_velocity.x);
    // printf("    y: %f \n", imu_data->angular_velocity.y);
    // printf("    z: %f \n", imu_data->angular_velocity.z);
    // printf("\n");
    // printf("orientation: \n");
    // printf("    x: %f \n", imu_data->orientation.x);
    // printf("    y: %f \n", imu_data->orientation.y);
    // printf("    z: %f \n", imu_data->orientation.z);
    // printf("    w: %f \n", imu_data->orientation.w);
    // printf("\n\n\n");

#ifndef USE_SEMAPHORE

#else
    sem_post(&metadata.data->write_sem);
#endif
    }

    munmap(header, sizeof(nv_shm_mgr_header_t));
    close(shm_fd);


}