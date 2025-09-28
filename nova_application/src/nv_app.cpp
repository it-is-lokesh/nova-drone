#include <iostream>
#include <thread>

#include <nova_processing/nv_drone.hpp>

int main(){
    nvDrone drone;

    std::thread update_imu(&nvStateEstimator::nvUpdateIMUThread, &drone.state_estimator);
    std::thread update_altimeter(&nvStateEstimator::nvUpdateAltimeterThread, &drone.state_estimator);
    std::thread update_navsat(&nvStateEstimator::nvUpdateNavSatThread, &drone.state_estimator);

    // while(1){
        // drone.state_estimator.nvPrintAccelerationWorld();
        // drone.state_estimator.nvPrintVelocityWorld();
        // drone.state_estimator.nvPrintPositionWorld();
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    update_imu.join();
    update_altimeter.join();
    update_navsat.join();

    return 0;


}
