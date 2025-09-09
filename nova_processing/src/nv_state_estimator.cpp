#include <nova_processing/nv_drone.hpp>

nvStateEstimator::nvStateEstimator() {
    nvInitializeIdentityMatrix(this->nvRotMat);
    pthread_mutex_init(&this->lock, NULL);
}

nv_status nvStateEstimator::nvSetTimePeriod(float32_t time_period) {
    this->nvTimePeriod = time_period;
    return 0;
}

nv_status nvStateEstimator::nvUpdateStateThread() {
    nv_shm_metadata_t metadata;
    nv_shm_mgr_header header;

    nv_imu_data imu_data;

    nvPingPongCounter counter(0, 10);
    int32_t shm_fd;

    snprintf(metadata.shm_name, SHM_NAME_MAX, "imu_shm\n");
    snprintf(metadata.pname, PROCESS_NAME_MAX, "orient_update\n");

    shm_fd = nvShmManager::nvMapShm(&header);

    nvShmManager::nvSetShmPtr(header, &metadata);

    imu_data = (nv_imu_data)metadata.data->data_ptr;

    while (1) {
#ifndef USE_SEMAPHORE

#else
        sem_wait(&metadata.data->read_sem);
#endif

        // Lock mutex
        pthread_mutex_lock(&this->lock);

        // Update state
        this->nvUpdateState(imu_data, counter.get());

        // Unlock mutex
        pthread_mutex_unlock(&this->lock);

        counter++;

#ifndef USE_SEMAPHORE

#else
        sem_post(&metadata.data->write_sem);
#endif
    }

    munmap(header, sizeof(nv_shm_mgr_header_t));
    close(shm_fd);

    return 0;
}

nv_status nvStateEstimator::nvUpdateState(nv_imu_data imu_data,
                                          int32_t loop_index) {
    nv_imu_data data = &imu_data[loop_index];

    // Update orientation
    this->nvUpdateOrientation(data, loop_index);

    // Update position
    this->nvUpdatePosition(data, loop_index);

    return 0;
}

nv_status nvStateEstimator::nvUpdateOrientation(nv_imu_data imu_data,
                                                int32_t loop_index) {
    this->nvOrientBody[0] = imu_data->angular_velocity.x;
    this->nvOrientBody[1] = imu_data->angular_velocity.y;
    this->nvOrientBody[2] = imu_data->angular_velocity.z;

    nv_mat<float64_t, 3, 3> omega_skew;

    omega_skew = nvGetSkewSymmetricMatrix(this->nvOrientBody);
    for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
            omega_skew[j][k] *= this->nvTimePeriod;
            if(j==k)omega_skew[j][j] += 1;
        }
    }

    nvMatrixMultiply(this->nvRotMat, omega_skew, this->nvRotMat);

    this->nvOrientWorld = nvGetOrientationFromRotMat(this->nvRotMat);

    if (loop_index % 10 == 0) {
        nvOrthonormalizeMatrix(this->nvRotMat);
    }
    return 0;
}

nv_status nvStateEstimator::nvUpdatePosition(nv_imu_data imu_data, int32_t loop_index) {
    this->nvAccelerationBody[0] = imu_data->linear_acceleration.x;
    this->nvAccelerationBody[1] = imu_data->linear_acceleration.y;
    this->nvAccelerationBody[2] = imu_data->linear_acceleration.z;

    nvMatrixMultiply(this->nvRotMat, this->nvAccelerationBody, this->nvAccelerationWorld);

    this->nvAccelerationWorld[2] -=9.8;

    for(int8_t i=0; i<3; i++){
        this->nvVelocityWorld[i] += this->nvAccelerationWorld[i] * this->nvTimePeriod;
    }

    for(int8_t i=0;i<3;i++){
        this->nvPositionWorld[i] += this->nvVelocityWorld[i] * this->nvTimePeriod;
    }

    return 0;
}

nv_vec<float64_t, 3> nvStateEstimator::nvGetOrientationWorld() {
    pthread_mutex_lock(&this->lock);
    nv_vec<float64_t, 3> ret = this->nvOrientWorld;
    pthread_mutex_unlock(&this->lock);
    return ret;
}

nv_vec<float64_t, 3> nvStateEstimator::nvGetOrientationBody() {
    pthread_mutex_lock(&this->lock);
    nv_vec<float64_t, 3> ret = this->nvOrientBody;
    pthread_mutex_unlock(&this->lock);
    return ret;
}

nv_mat<float64_t, 3, 3> nvStateEstimator::nvGetRotationMatrix() {
    pthread_mutex_lock(&this->lock);
    nv_mat<float64_t, 3, 3> ret = this->nvRotMat;
    pthread_mutex_unlock(&this->lock);
    return ret;
}

nv_vec<float64_t, 3> nvStateEstimator::nvGetAccelerationBody() {
    pthread_mutex_lock(&this->lock);
    nv_vec<float64_t, 3> ret = this->nvAccelerationBody;
    pthread_mutex_unlock(&this->lock);
    return ret;
}

nv_vec<float64_t, 3> nvStateEstimator::nvGetAccelerationWorld() {
    pthread_mutex_lock(&this->lock);
    nv_vec<float64_t, 3> ret = this->nvAccelerationWorld;
    pthread_mutex_unlock(&this->lock);
    return ret;
}

nv_vec<float64_t, 3> nvStateEstimator::nvGetVelocityWorld() {
    pthread_mutex_lock(&this->lock);
    nv_vec<float64_t, 3> ret = this->nvVelocityWorld;
    pthread_mutex_unlock(&this->lock);
    return ret;
}

nv_vec<float64_t, 3> nvStateEstimator::nvGetPositionWorld() {
    pthread_mutex_lock(&this->lock);
    nv_vec<float64_t, 3> ret = this->nvPositionWorld;
    pthread_mutex_unlock(&this->lock);
    return ret;
}

nv_status nvStateEstimator::nvPrintOrientationWorld() {
    pthread_mutex_lock(&this->lock);
    printf("OrientationWorld: %f %f %f \n", this->nvOrientWorld[0], this->nvOrientWorld[1],
           this->nvOrientWorld[2]);
    pthread_mutex_unlock(&this->lock);
    return 0;
}

nv_status nvStateEstimator::nvPrintOrientationBody() {
    pthread_mutex_lock(&this->lock);
    printf("OrientationBody: %f %f %f \n", this->nvOrientBody[0], this->nvOrientBody[1],
           this->nvOrientBody[2]);
    pthread_mutex_unlock(&this->lock);
    return 0;
}

nv_status nvStateEstimator::nvPrintRotationMatrix() {
    pthread_mutex_lock(&this->lock);
    // printf("RotMat: %f %f %f \n", nvOrientWorld[0], nvOrientWorld[1],
    //        nvOrientWorld[2]);

    // TODO: Complete this function
    printf("nv_status nvStateEstimator::nvPrintRotationMatrix() function not implemented \n");
    pthread_mutex_unlock(&this->lock);
    return 0;
}

nv_status nvStateEstimator::nvPrintAccelerationBody() {
    pthread_mutex_lock(&this->lock);
    printf("AccelerationBody: %f %f %f \n", this->nvAccelerationBody[0], this->nvAccelerationBody[1],
           this->nvAccelerationBody[2]);
    pthread_mutex_unlock(&this->lock);
    return 0;
}

nv_status nvStateEstimator::nvPrintAccelerationWorld() {
    pthread_mutex_lock(&this->lock);
    printf("AccelerationWorld: %f %f %f \n", this->nvAccelerationWorld[0], this->nvAccelerationWorld[1],
           this->nvAccelerationWorld[2]);
    pthread_mutex_unlock(&this->lock);
    return 0;
}

nv_status nvStateEstimator::nvPrintVelocityWorld() {
    pthread_mutex_lock(&this->lock);
    printf("VelocityWorld: %f %f %f \n", nvVelocityWorld[0], nvVelocityWorld[1],
           nvVelocityWorld[2]);
    pthread_mutex_unlock(&this->lock);
    return 0;
}

nv_status nvStateEstimator::nvPrintPositionWorld() {
    pthread_mutex_lock(&this->lock);
    printf("PositionWorld: %f %f %f \n", nvPositionWorld[0], nvPositionWorld[1],
           nvPositionWorld[2]);
    pthread_mutex_unlock(&this->lock);
    return 0;
}
