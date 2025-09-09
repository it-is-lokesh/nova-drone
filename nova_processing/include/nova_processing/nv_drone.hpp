#include <nova_processing/nv.hpp>

class nvStateEstimator {
private:
    float32_t nvTimePeriod;

    pthread_mutex_t lock;

    nv_vec<float64_t, 3> nvOrientBody;
    nv_vec<float64_t, 3> nvOrientWorld;
    nv_mat<float64_t, 3, 3> nvRotMat;

    nv_vec<float64_t, 3> nvAccelerationBody;
    nv_vec<float64_t, 3> nvAccelerationWorld;
    nv_vec<float64_t, 3> nvVelocityWorld;
    nv_vec<float64_t, 3> nvPositionWorld;

public:
    nvStateEstimator();

    nv_status nvSetTimePeriod(float32_t time_period);

    nv_status nvUpdateStateThread();

    nv_status nvUpdateState(nv_imu_data imu_data, int32_t loop_index);

    nv_status nvUpdateOrientation(nv_imu_data imu_data, int32_t loop_index);

    nv_status nvUpdatePosition(nv_imu_data imu_data, int32_t loop_index);

    nv_vec<float64_t, 3> nvGetOrientationWorld();

    nv_vec<float64_t, 3> nvGetOrientationBody();

    nv_mat<float64_t, 3, 3> nvGetRotationMatrix();

    nv_vec<float64_t, 3> nvGetAccelerationBody();

    nv_vec<float64_t, 3> nvGetAccelerationWorld();

    nv_vec<float64_t, 3> nvGetVelocityWorld();

    nv_vec<float64_t, 3> nvGetPositionWorld();

    nv_status nvPrintOrientationWorld();

    nv_status nvPrintOrientationBody();

    nv_status nvPrintRotationMatrix();

    nv_status nvPrintAccelerationBody();

    nv_status nvPrintAccelerationWorld();

    nv_status nvPrintVelocityWorld();

    nv_status nvPrintPositionWorld();
};

class nvDrone {
private:
public:
    nvStateEstimator state_estimator;

    nvDrone() {
        state_estimator.nvSetTimePeriod(0.01);
    }

    nvStateEstimator *nvGetStateEstimator();
};
