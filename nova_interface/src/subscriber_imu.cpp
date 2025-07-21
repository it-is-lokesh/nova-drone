#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <nova_processing/nv_types.hpp>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

class SubscriberIMU : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    nv_shm_obj_t shm;
    nv_shm_data shm_data;
    nv_imu_data imu_data;
    int loop;
public:
    SubscriberIMU() : Node("node_subscriber_imu"){
        shm.shm_id = SHM_SENSOR_IMU;
        shm.shm_size = sizeof(nv_shm_data_t) + sizeof(nv_imu_data_t);
        snprintf(shm.shm_name, sizeof(shm.shm_name), "/shm_id_%d", shm.shm_id);
        int shm_fd = shm_open(shm.shm_name, O_CREAT | O_RDWR, 0660);
        if(shm_fd == -1) perror("shm_open failed");

        if (ftruncate(shm_fd, shm.shm_size) == -1) perror("ftruncate");

        shm.data = (nv_shm_data)mmap(0, shm.shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if(shm.data == MAP_FAILED) perror("mmap failed");

        shm.data->data_ptr = (char *)shm.data + sizeof(nv_shm_data_t);
        shm_data = (nv_shm_data)shm.data;
        imu_data = (nv_imu_data)shm_data->data_ptr;

        loop = 0;

#ifndef USE_SEMAPHORE
        pthread_mutexattr_init(&shm.attr);
        pthread_mutexattr_setpshared(&shm.attr, PTHREAD_PROCESS_SHARED);
        pthread_mutex_init(&shm.lock, &shm.attr);
        pthread_cond_init(&shm.cond);
        shm.flag = 0;
#else
        sem_init(&shm_data->write_sem, 1, 1);
        sem_init(&shm_data->read_sem, 1, 0);
#endif

        auto topic_callback = [this](sensor_msgs::msg::Imu::UniquePtr msg) -> void {
#ifndef USE_SEMAPHORE
            pthread_mutex_lock(&shm.lock);
#else
            sem_wait(&shm_data->write_sem);
#endif
            printf("loop: %d \n", loop++);

            imu_data->sec = msg->header.stamp.sec;
            imu_data->nsec = msg->header.stamp.nanosec;

            imu_data->linear_acceleration.x = msg->linear_acceleration.x;
            imu_data->linear_acceleration.y = msg->linear_acceleration.y;
            imu_data->linear_acceleration.z = msg->linear_acceleration.z;
            memcpy(imu_data->linear_acceleration.covariance, msg->linear_acceleration_covariance.data(), 
                   msg->linear_acceleration_covariance.size());

            imu_data->angular_velocity.x = msg->angular_velocity.x;
            imu_data->angular_velocity.y = msg->angular_velocity.y;
            imu_data->angular_velocity.z = msg->angular_velocity.z;
            memcpy(imu_data->angular_velocity.covariance, msg->angular_velocity_covariance.data(), 
                   msg->angular_velocity_covariance.size());

            imu_data->orientation.x = msg->orientation.x;
            imu_data->orientation.y = msg->orientation.y;
            imu_data->orientation.z = msg->orientation.z;
            memcpy(imu_data->orientation.covariance, msg->orientation_covariance.data(), 
                   msg->angular_velocity_covariance.size());

#ifndef USE_SEMAPHORE
            shm.flag = 1;
            pthread_mutex_unlock(&shm.lock);
#else
            sem_post(&shm_data->read_sem);
#endif
        };
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("/nova/imu", 10, topic_callback);
    }

    ~SubscriberIMU(){
        shm_unlink(shm.shm_name);
    }
};


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberIMU>());
    rclcpp::shutdown();
    
    return 0;
}