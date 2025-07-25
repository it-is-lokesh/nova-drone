#include <stdio.h>

#include <nova_processing/nv.hpp>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

int main(){
    nv_shm_obj_t shm;
    nv_shm_data shm_data;
    nv_imu_data imu_data;

    shm.shm_id = SHM_SENSOR_IMU;
    shm.shm_size = sizeof(nv_shm_data_t) + sizeof(nv_imu_data_t);
    snprintf(shm.shm_name, sizeof(shm.shm_name), "/shm_id_%d", shm.shm_id);
    int shm_fd = shm_open(shm.shm_name, O_RDWR, 0660);
    if(shm_fd == -1) perror("shm_open failed");

    shm.data = (nv_shm_data)mmap(0, shm.shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if(shm.data == MAP_FAILED) perror("mmap failed");

    shm.data->data_ptr = (char *)shm.data + sizeof(nv_shm_data_t);
    shm_data = (nv_shm_data)shm.data;
    imu_data = (nv_imu_data)shm_data->data_ptr;

    // Open file to store the data in a csv file
    FILE *fp;

    fp = fopen("imu_logs.csv", "a+");

    fprintf(fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", "sec", "nsec", "lin_acc.x", "lin_acc.y", "lin_acc.z", 
                                            "ang_vel.x", "ang_vel.y", "ang_vel.z",
                                            "orient.x", "orient.y", "orient.z", "orient.w"
                                            );

    int loop = 0;
    while(1){
#ifndef USE_SEMAPHORE

#else
    sem_wait(&shm_data->read_sem);
#endif
    printf("loop: %d \n", loop++);

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
    sem_post(&shm_data->write_sem);
#endif
    }

#ifndef USE_SEMAPHORE
    
#else
    
#endif


}