#include <nova_processing/nv.hpp>

// #define LOG_IMU_DATA

class nvKalmanFilter {
private:
    float32_t nvTimePeriod;
    uint32_t nvNumStates;

    nv_vec<float64_t, 12> nvStateEstimate;              // x_k|k;
    nv_vec<float64_t, 12> nvStatePrediction;            // x_k|k-1;
    nv_mat<float64_t, 12, 12> nvStateTransition;        // phi
    nv_mat<float64_t, 12, 12> nvStateTransitionT;       // phi^T
    nv_mat<float64_t, 12, 12> nvProcessNoiseCov;        // Q
    nv_mat<float64_t, 12, 12> nvStateCovEstimate;       // P_k|k
    nv_mat<float64_t, 12, 12> nvStateCovPrediction;     // P_k|k-1
    nv_mat<float64_t, 3, 3> nvMeasurementNoiseCov;      // R
    nv_mat<float64_t, 3, 12> nvMeasurementMatrix;       // H
    nv_mat<float64_t, 12, 3> nvMeasurementMatrixT;       // H^T
    nv_vec<float64_t, 3> nvInnovation;                  // y
    nv_mat<float64_t, 3, 3> nvInnovationCov;            // S
    nv_mat<float64_t, 12, 3> nvKalmanGain;              // K
    

public:
    nvKalmanFilter(float32_t time_period);

    nv_status nvKFPredict();

    nv_status nvKFUpdate(nv_vec<float64_t, 3> &measurement);

    nv_status nvStep(nv_vec<float64_t, 3> &measurement);

    nv_status nvSetTimePeriod(float32_t time_period);

    nv_vec<float64_t, 3> nvGetPositionEstimate();

    nv_vec<float64_t, 3> nvGetVelocityEstimate();

    nv_vec<float64_t, 3> nvGetAccelerationEstimate();
};

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

    nvKalmanFilter acc_kf;

#ifdef LOG_IMU_DATA
    FILE *fp;
#endif

public:
    nvStateEstimator(float32_t time_period) : acc_kf(0.01) {
        nvInitializeIdentityMatrix(this->nvRotMat);
        pthread_mutex_init(&this->lock, NULL);
    }

    nv_status nvUpdateIMUThread();

    nv_status nvUpdateIMU(nv_imu_data imu_data, int32_t loop_index);

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

    nvDrone() : state_estimator(0.01) {}

    nvStateEstimator *nvGetStateEstimator();
};
