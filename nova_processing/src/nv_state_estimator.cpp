#include <nova_processing/nv_drone.hpp>

nv_status nvStateEstimator::nvUpdateIMUThread() {
    nv_shm_metadata_t metadata;

    nv_imu_data imu_data;

    nvPingPongCounter counter(0, 10);

    snprintf(metadata.shm_name, SHM_NAME_MAX, "imu_shm\n");
    snprintf(metadata.pname, PROCESS_NAME_MAX, "orient_update\n");

    nvShmManager::nvSetShmPtr(header, &metadata);

    imu_data = (nv_imu_data)metadata.data->data_ptr;

#ifdef LOG_IMU_DATA
    // Open file to store the data in a csv file

    fp = fopen("imu_logs.csv", "a+");

    fprintf(fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", "sec", "nsec", "lin_acc.x", "lin_acc.y", "lin_acc.z", 
                                            "ang_vel.x", "ang_vel.y", "ang_vel.z",
                                            "orient.x", "orient.y", "orient.z", "orient.w"
                                            );
#endif

    while (1) {
#ifndef USE_SEMAPHORE

#else
        sem_wait(&metadata.data->read_sem);
#endif

        // Lock mutex
        pthread_mutex_lock(&this->lock_imu);
        // Update state
        this->nvUpdateIMU(&imu_data[counter.get_value()%counter.get_size()], counter.get_value());

        // Unlock mutex
        pthread_mutex_unlock(&this->lock_imu);

        counter++;

#ifndef USE_SEMAPHORE

#else
        sem_post(&metadata.data->write_sem);
#endif
    }

    return NV_SUCCESS;
}

nv_status nvStateEstimator::nvUpdateAltimeterThread() {
    nv_shm_metadata_t metadata;

    nv_altimeter_data altimeter_data;

    nvPingPongCounter counter(0, 9);

    snprintf(metadata.shm_name, SHM_NAME_MAX, "altimeter_shm\n");
    snprintf(metadata.pname, PROCESS_NAME_MAX, "altimeter_update\n");

    nvShmManager::nvSetShmPtr(header, &metadata);

    altimeter_data = (nv_altimeter_data)metadata.data->data_ptr;

    while (1) {
#ifndef USE_SEMAPHORE

#else
        sem_wait(&metadata.data->read_sem);
#endif

        // Lock mutex
        pthread_mutex_lock(&this->lock_altimeter);

        // Update state
        this->nvUpdateAltitude(&altimeter_data[counter.get_value()%counter.get_size()], counter.get_value());

        // Unlock mutex
        pthread_mutex_unlock(&this->lock_altimeter);

        counter++;
#ifndef USE_SEMAPHORE

#else
        sem_post(&metadata.data->write_sem);
#endif
    }
    
}

nv_status nvStateEstimator::nvUpdateIMU(nv_imu_data imu_data,
                                                int32_t loop_index) {
#ifdef LOG_IMU_DATA
    // Log data to file
    fprintf(fp, "%d,%d,%.17f,%.17f,%.17f,%.17f,%.17f,%.17f,%.17f,%.17f,%.17f,%.17f\n", 
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
#endif

    this->nvOrientBody[0] = imu_data->angular_velocity.x;
    this->nvOrientBody[1] = imu_data->angular_velocity.y;
    this->nvOrientBody[2] = imu_data->angular_velocity.z;

    nv_mat<float64_t, 3, 3> omega_skew;

    omega_skew = nvGetSkewSymmetricMatrix(this->nvOrientBody);
    for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
            omega_skew[j][k] *= this->nvTimePeriodIMU;
            if(j==k)omega_skew[j][j] += 1;
        }
    }

    this->nvRotMat = this->nvRotMat * omega_skew;

    this->nvOrientWorld = nvGetOrientationFromRotMat(this->nvRotMat);

    if (loop_index % 10 == 0) {
        nvOrthonormalizeMatrix(this->nvRotMat);
    }

    nv_vec<float64_t, 3> nvAccTransformed;
    nvAccTransformed[0] = imu_data->linear_acceleration.x;
    nvAccTransformed[1] = imu_data->linear_acceleration.y;
    nvAccTransformed[2] = imu_data->linear_acceleration.z;

    nvAccTransformed = this->nvRotMat * nvAccTransformed;

    this->nvAccelerationWorld = nvAccTransformed;
    return NV_SUCCESS;
}

nv_status nvStateEstimator::nvUpdateAltitude(nv_altimeter_data altimeter_data, int32_t loop_index) {
    float64_t prev_height = this->nvPositionWorld[2];
    this->nvPositionWorld[2] = altimeter_data->vertical_position;
    printf("Altitude: %f \n", this->nvPositionWorld[2]);
    this->nvVelocityWorld[2] = (this->nvPositionWorld[2] - prev_height) / this->nvTimePeriodAltimeter;
    return NV_SUCCESS;
}

nv_status nvStateEstimator::nvUpdatePosition(nv_imu_data imu_data, int32_t loop_index) {
    nv_vec<float64_t, 3> nvAccTransformed;
    nvAccTransformed[0] = imu_data->linear_acceleration.x;
    nvAccTransformed[1] = imu_data->linear_acceleration.y;
    nvAccTransformed[2] = imu_data->linear_acceleration.z;

    nvAccTransformed = this->nvRotMat * nvAccTransformed;

    this->acc_kf.nvStep(nvAccTransformed);

    this->nvPositionWorld = this->acc_kf.nvGetPositionEstimate();
    this->nvVelocityWorld = this->acc_kf.nvGetVelocityEstimate();
    this->nvAccelerationWorld = this->acc_kf.nvGetAccelerationEstimate();

    return NV_SUCCESS;
}

nv_vec<float64_t, 3> nvStateEstimator::nvGetOrientationWorld() {
    pthread_mutex_lock(&this->lock_imu);
    nv_vec<float64_t, 3> ret = this->nvOrientWorld;
    pthread_mutex_unlock(&this->lock_imu);
    return ret;
}

nv_vec<float64_t, 3> nvStateEstimator::nvGetOrientationBody() {
    pthread_mutex_lock(&this->lock_imu);
    nv_vec<float64_t, 3> ret = this->nvOrientBody;
    pthread_mutex_unlock(&this->lock_imu);
    return ret;
}

nv_mat<float64_t, 3, 3> nvStateEstimator::nvGetRotationMatrix() {
    pthread_mutex_lock(&this->lock_imu);
    nv_mat<float64_t, 3, 3> ret = this->nvRotMat;
    pthread_mutex_unlock(&this->lock_imu);
    return ret;
}

nv_vec<float64_t, 3> nvStateEstimator::nvGetAccelerationBody() {
    pthread_mutex_lock(&this->lock_imu);
    nv_vec<float64_t, 3> ret = this->nvAccelerationBody;
    pthread_mutex_unlock(&this->lock_imu);
    return ret;
}

nv_vec<float64_t, 3> nvStateEstimator::nvGetAccelerationWorld() {
    pthread_mutex_lock(&this->lock_imu);
    nv_vec<float64_t, 3> ret = this->nvAccelerationWorld;
    pthread_mutex_unlock(&this->lock_imu);
    return ret;
}

nv_vec<float64_t, 3> nvStateEstimator::nvGetVelocityWorld() {
    // pthread_mutex_lock(&this->lock);
    nv_vec<float64_t, 3> ret = this->nvVelocityWorld;
    // pthread_mutex_unlock(&this->lock);
    return ret;
}

nv_vec<float64_t, 3> nvStateEstimator::nvGetPositionWorld() {
    // pthread_mutex_lock(&this->lock);
    nv_vec<float64_t, 3> ret = this->nvPositionWorld;
    // pthread_mutex_unlock(&this->lock);
    return ret;
}

nv_status nvStateEstimator::nvPrintOrientationWorld() {
    pthread_mutex_lock(&this->lock_imu);
    printf("OrientationWorld: %f %f %f \n", this->nvOrientWorld[0], this->nvOrientWorld[1],
           this->nvOrientWorld[2]);
    pthread_mutex_unlock(&this->lock_imu);
    return NV_SUCCESS;
}

nv_status nvStateEstimator::nvPrintOrientationBody() {
    pthread_mutex_lock(&this->lock_imu);
    printf("OrientationBody: %f %f %f \n", this->nvOrientBody[0], this->nvOrientBody[1],
           this->nvOrientBody[2]);
    pthread_mutex_unlock(&this->lock_imu);
    return NV_SUCCESS;
}

nv_status nvStateEstimator::nvPrintRotationMatrix() {
    pthread_mutex_lock(&this->lock_imu);
    // printf("RotMat: %f %f %f \n", nvOrientWorld[0], nvOrientWorld[1],
    //        nvOrientWorld[2]);

    // TODO: Complete this function
    printf("nv_status nvStateEstimator::nvPrintRotationMatrix() function not implemented \n");
    pthread_mutex_unlock(&this->lock_imu);
    return NV_SUCCESS;
}

nv_status nvStateEstimator::nvPrintAccelerationBody() {
    pthread_mutex_lock(&this->lock_imu);
    printf("AccelerationBody: %f %f %f \n", this->nvAccelerationBody[0], this->nvAccelerationBody[1],
           this->nvAccelerationBody[2]);
    pthread_mutex_unlock(&this->lock_imu);
    return NV_SUCCESS;
}

nv_status nvStateEstimator::nvPrintAccelerationWorld() {
    pthread_mutex_lock(&this->lock_imu);
    printf("AccelerationWorld: %0.17f %0.17f %0.17f \n", this->nvAccelerationWorld[0], this->nvAccelerationWorld[1],
           this->nvAccelerationWorld[2]);
    pthread_mutex_unlock(&this->lock_imu);
    return NV_SUCCESS;
}

nv_status nvStateEstimator::nvPrintVelocityWorld() {
    // pthread_mutex_lock(&this->lock);
    printf("VelocityWorld: %f %f %f \n", nvVelocityWorld[0], nvVelocityWorld[1],
           nvVelocityWorld[2]);
    // pthread_mutex_unlock(&this->lock);
    return NV_SUCCESS;
}

nv_status nvStateEstimator::nvPrintPositionWorld() {
    // pthread_mutex_lock(&this->lock);
    printf("PositionWorld: %f %f %f \n", nvPositionWorld[0], nvPositionWorld[1],
           nvPositionWorld[2]);
    // pthread_mutex_unlock(&this->lock);
    return NV_SUCCESS;
}
