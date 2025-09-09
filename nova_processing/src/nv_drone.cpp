#include <nova_processing/nv_drone.hpp>

nvStateEstimator *nvDrone::nvGetStateEstimator(){
    return &this->state_estimator;
}