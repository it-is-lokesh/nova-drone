#include <nova_processing/nv_drone.hpp>

nvKalmanFilter::nvKalmanFilter(float32_t time_period) {
    this->nvNumStates = 12;
    this->nvTimePeriod = time_period;

    // Set state transition matrix
    nv_mat<float64_t, 3, 3> identity_mat;
    nvInitializeIdentityMatrix(identity_mat);

    nvSetMatrixUsingSlices(this->nvStateTransition, identity_mat,
                           nv_vec<uint32_t, 2>{0, 0});
    nvSetMatrixUsingSlices(this->nvStateTransition, identity_mat,
                           nv_vec<uint32_t, 2>{3, 3});
    nvSetMatrixUsingSlices(this->nvStateTransition, identity_mat,
                           nv_vec<uint32_t, 2>{6, 6});
    nvSetMatrixUsingSlices(this->nvStateTransition, identity_mat,
                           nv_vec<uint32_t, 2>{9, 9});

    nv_mat<float64_t, 3, 3> time_identity_mat =
        identity_mat * this->nvTimePeriod;
    nvSetMatrixUsingSlices(this->nvStateTransition, time_identity_mat,
                           nv_vec<uint32_t, 2>{0, 3});
    nvSetMatrixUsingSlices(this->nvStateTransition, time_identity_mat,
                           nv_vec<uint32_t, 2>{3, 6});

    nv_mat<float64_t, 3, 3> half_time_sq_identity_mat =
        time_identity_mat * (0.5 * this->nvTimePeriod);
    nvSetMatrixUsingSlices(this->nvStateTransition, half_time_sq_identity_mat,
                           nv_vec<uint32_t, 2>{0, 6});

    this->nvStateTransitionT = nvTransposeMatrix(this->nvStateTransition);

    // Set process noise covariance matrix
    FILE *fp;
    nv_mat<float64_t, 3, 3> Q_c_a;
    fp = fopen("Q_c_a.bin", "rb");
    fread(Q_c_a.begin, sizeof(float64_t), 3 * 3, fp);
    fclose(fp);

    nv_mat<float64_t, 3, 3> Q_c_b;
    fp = fopen("Q_c_b.bin", "rb");
    fread(Q_c_b.begin, sizeof(float64_t), 3 * 3, fp);
    fclose(fp);

    fp = fopen("R.bin", "rb");
    fread(this->nvMeasurementNoiseCov.begin, sizeof(float64_t), 3 * 3, fp);
    fclose(fp);

    nv_mat<float64_t, 3, 3> Q_aa = Q_c_a * this->nvTimePeriod;
    nv_mat<float64_t, 3, 3> Q_bb = Q_c_b * this->nvTimePeriod;
    nv_mat<float64_t, 3, 3> Q_va = Q_aa * (0.5 * this->nvTimePeriod);
    nv_mat<float64_t, 3, 3> Q_vv = Q_va * (2 * this->nvTimePeriod / 3);
    nv_mat<float64_t, 3, 3> Q_pa = Q_vv * 0.5;
    nv_mat<float64_t, 3, 3> Q_pv = Q_pa * (3 * this->nvTimePeriod / 4);
    nv_mat<float64_t, 3, 3> Q_pp = Q_pv * (2 * this->nvTimePeriod / 5);

    nvSetMatrixUsingSlices(this->nvProcessNoiseCov, Q_aa,
                           nv_vec<uint32_t, 2>{6, 6});
    nvSetMatrixUsingSlices(this->nvProcessNoiseCov, Q_bb,
                           nv_vec<uint32_t, 2>{9, 9});
    nvSetMatrixUsingSlices(this->nvProcessNoiseCov, Q_va,
                           nv_vec<uint32_t, 2>{3, 6});
    nvSetMatrixUsingSlices(this->nvProcessNoiseCov, Q_vv,
                           nv_vec<uint32_t, 2>{3, 3});
    nvSetMatrixUsingSlices(this->nvProcessNoiseCov, Q_pa,
                           nv_vec<uint32_t, 2>{0, 6});
    nvSetMatrixUsingSlices(this->nvProcessNoiseCov, Q_pv,
                           nv_vec<uint32_t, 2>{0, 3});
    nvSetMatrixUsingSlices(this->nvProcessNoiseCov, Q_pp,
                           nv_vec<uint32_t, 2>{0, 0});

    nv_mat<float64_t, 3, 3> Q_va_T = Q_va.get_transpose();
    nv_mat<float64_t, 3, 3> Q_pa_T = Q_pa.get_transpose();
    nv_mat<float64_t, 3, 3> Q_pv_T = Q_pv.get_transpose();
    nvSetMatrixUsingSlices(this->nvProcessNoiseCov, Q_va_T,
                           nv_vec<uint32_t, 2>{6, 3});
    nvSetMatrixUsingSlices(this->nvProcessNoiseCov, Q_pa_T,
                           nv_vec<uint32_t, 2>{6, 0});
    nvSetMatrixUsingSlices(this->nvProcessNoiseCov, Q_pv_T,
                           nv_vec<uint32_t, 2>{3, 0});

    // Set measurement matrix
    nvSetMatrixUsingSlices(this->nvMeasurementMatrix, identity_mat,
                           nv_vec<uint32_t, 2>{0, 6});
    nvSetMatrixUsingSlices(this->nvMeasurementMatrix, identity_mat,
                           nv_vec<uint32_t, 2>{0, 9});

    this->nvMeasurementMatrixT = nvTransposeMatrix(this->nvMeasurementMatrix);

    // Set State Covariance Estimate matrix
    nvInitializeIdentityMatrix(this->nvStateCovEstimate);
    this->nvStateCovEstimate = this->nvStateCovEstimate * 0.1;
}

nv_status nvKalmanFilter::nvKFPredict() {
    this->nvStatePrediction = this->nvStateTransition * this->nvStateEstimate;
    // printf("Predicted State: \n");
    // for(int i=0; i<12; i++){
    //     for(int j=0; j<12; j++){
    //         printf("%0.5f ", this->nvStateCovEstimate[i][j]);
    //     }
    //     printf("\n");
    // }
    // printf("\n");
    // int temp;
    // scanf("%d", &temp);

    // print nvstate prediction vector
    // for(int i=0; i<12; i++){
    //     printf("%0.5f ", this->nvStatePrediction[i]);
    // }
    // printf("\n");
    // int temp;
    // scanf("%d", &temp);
    this->nvStateCovPrediction = this->nvStateTransition *
                                     this->nvStateCovEstimate *
                                     this->nvStateTransitionT +
                                 this->nvProcessNoiseCov;
    return NV_SUCCESS;
}

nv_status nvKalmanFilter::nvKFUpdate(nv_vec<float64_t, 3> &measurement) {
    nv_mat<float64_t, 12, 12> identity_mat;
    nvInitializeIdentityMatrix(identity_mat);

    nv_vec<float64_t, 3> predicted_meas = 
        this->nvMeasurementMatrix * this->nvStatePrediction;

    // print predicted measurement
    // printf("Predicted Measurement: ");
    // for(int i=0; i<3; i++){
    //     printf("%0.5f ", predicted_meas[i]);
    // }
    // printf("\n");

    this->nvInnovation = measurement - nvConstants::nvGravity -
                         predicted_meas;
    this->nvInnovationCov = this->nvMeasurementMatrix *
                                this->nvStateCovPrediction *
                                this->nvMeasurementMatrixT +
                            this->nvMeasurementNoiseCov;
    this->nvKalmanGain = this->nvStateCovPrediction *
                         this->nvMeasurementMatrixT *
                         this->nvInnovationCov.get_inverse();

    this->nvStateEstimate =
        this->nvStatePrediction + this->nvKalmanGain * this->nvInnovation;
    this->nvStateCovEstimate =
        (identity_mat - this->nvKalmanGain * this->nvMeasurementMatrix) *
            this->nvStateCovPrediction *
            (identity_mat - this->nvKalmanGain * this->nvMeasurementMatrix)
                .get_transpose() +
        this->nvKalmanGain * this->nvMeasurementNoiseCov *
            this->nvKalmanGain.get_transpose();
    return NV_SUCCESS;
}

nv_status nvKalmanFilter::nvStep(nv_vec<float64_t, 3> &measurement) {
    this->nvKFPredict();
    this->nvKFUpdate(measurement);
    return NV_SUCCESS;
}

nv_status nvKalmanFilter::nvSetTimePeriod(float32_t time_period) {
    this->nvTimePeriod = time_period;
    return NV_SUCCESS;
}

nv_vec<float64_t, 3> nvKalmanFilter::nvGetPositionEstimate() {
    nv_vec<float64_t, 3> pos_estimate;
    pos_estimate[0] = this->nvStateEstimate[0];
    pos_estimate[1] = this->nvStateEstimate[1];
    pos_estimate[2] = this->nvStateEstimate[2];
    return pos_estimate;
}

nv_vec<float64_t, 3> nvKalmanFilter::nvGetVelocityEstimate() {
    nv_vec<float64_t, 3> vel_estimate;
    vel_estimate[0] = this->nvStateEstimate[3];
    vel_estimate[1] = this->nvStateEstimate[4];
    vel_estimate[2] = this->nvStateEstimate[5];
    return vel_estimate;
}

nv_vec<float64_t, 3> nvKalmanFilter::nvGetAccelerationEstimate() {
    nv_vec<float64_t, 3> acc_estimate;
    acc_estimate[0] = this->nvStateEstimate[6];
    acc_estimate[1] = this->nvStateEstimate[7];
    acc_estimate[2] = this->nvStateEstimate[8];
    return acc_estimate;
}