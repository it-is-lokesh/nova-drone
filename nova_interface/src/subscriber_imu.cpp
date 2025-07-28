#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <nova_processing/nv.hpp>


class SubscriberIMU : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    nv_shm_metadata_t metadata;
    nv_shm_mgr_header header;
    nv_imu_data imu_data;
    int loop;
    int count;
    int shm_fd;
public:
    SubscriberIMU() : Node("node_subscriber_imu"){
        count = 10;

        snprintf(metadata.shm_name, SHM_NAME_MAX, "test_shm_1\n");
        snprintf(metadata.pname, SHM_NAME_MAX, "test_process_1\n");

        shm_fd = nvShmManager::nvMapShm(&header);

        nvShmManager::nvGetShmPtr(header, sizeof(int), count, &metadata);
        
        imu_data = (nv_imu_data)metadata.data->data_ptr;

        loop = 0;

#ifndef USE_SEMAPHORE
        pthread_mutexattr_init(&metadata.data->attr);
        pthread_mutexattr_setpshared(&metadata.data->attr, PTHREAD_PROCESS_SHARED);
        pthread_mutex_init(&metadata.data->lock, &metadata.data->attr);
        pthread_cond_init(&metadata.data->cond);
        metadata.data->flag = 0;
#else
        sem_init(&metadata.data->read_sem, 1, 0);
        sem_init(&metadata.data->write_sem, 1, count);
#endif

        auto topic_callback = [this](sensor_msgs::msg::Imu::UniquePtr msg) -> void {
#ifndef USE_SEMAPHORE
            pthread_mutex_lock(&metadata.data->lock);
#else
            sem_wait(&metadata.data->write_sem);
#endif
            printf("loop: %d \n", loop++);

            imu_data[loop%count].sec = msg->header.stamp.sec;
            imu_data[loop%count].nsec = msg->header.stamp.nanosec;

            imu_data[loop%count].linear_acceleration.x = msg->linear_acceleration.x;
            imu_data[loop%count].linear_acceleration.y = msg->linear_acceleration.y;
            imu_data[loop%count].linear_acceleration.z = msg->linear_acceleration.z;
            memcpy(imu_data[loop%count].linear_acceleration.covariance, msg->linear_acceleration_covariance.data(), 
                   msg->linear_acceleration_covariance.size());

            imu_data[loop%count].angular_velocity.x = msg->angular_velocity.x;
            imu_data[loop%count].angular_velocity.y = msg->angular_velocity.y;
            imu_data[loop%count].angular_velocity.z = msg->angular_velocity.z;
            memcpy(imu_data[loop%count].angular_velocity.covariance, msg->angular_velocity_covariance.data(), 
                   msg->angular_velocity_covariance.size());

            imu_data[loop%count].orientation.x = msg->orientation.x;
            imu_data[loop%count].orientation.y = msg->orientation.y;
            imu_data[loop%count].orientation.z = msg->orientation.z;
            memcpy(imu_data[loop%count].orientation.covariance, msg->orientation_covariance.data(), 
                   msg->angular_velocity_covariance.size());

#ifndef USE_SEMAPHORE
            metadata.data->flag = 1;
            pthread_cond_signal(&metadata.data->cond);
            pthread_mutex_unlock(&metadata.data->lock);
#else
            sem_post(&metadata.data->read_sem);
#endif
        };
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("/nova/imu", 10, topic_callback);
    }

    ~SubscriberIMU(){
        munmap(header, sizeof(nv_shm_mgr_header_t));
        close(shm_fd);
    }
};


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberIMU>());
    rclcpp::shutdown();
    
    return 0;
}