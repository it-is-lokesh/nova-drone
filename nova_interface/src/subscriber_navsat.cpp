#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <gps_msgs/msg/gps_fix.hpp>

#include <nova_processing/nv.hpp>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

class Subscribernavsat : public rclcpp::Node {
private:
    rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr subscription_;
    nv_shm_metadata_t metadata;
    nv_shm_mgr_header header;
    nv_navsat_data navsat_data;
    int loop;
    int count;
    int shm_fd;
public:
    Subscribernavsat() : Node("node_subscriber_navsat"){
        count = 10;
        snprintf(metadata.shm_name, SHM_NAME_MAX, "navsat_shm\n");
        snprintf(metadata.pname, PROCESS_NAME_MAX, "navsat_interface\n");

        shm_fd = nvShmManager::nvMapShm(&header);

        nvShmManager::nvGetShmPtr(header, sizeof(nv_navsat_data_t), count, &metadata);

        navsat_data = (nv_navsat_data)metadata.data->data_ptr;

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

        auto topic_callback = [this](gps_msgs::msg::GPSFix::UniquePtr msg) -> void {
#ifndef USE_SEMAPHORE
            pthread_mutex_lock(&metadata.data->lock);
#else
            sem_wait(&metadata.data->write_sem);
#endif
            navsat_data[loop%count].latitude = msg->latitude;
            navsat_data[loop%count].longitude = msg->longitude;
            navsat_data[loop%count].altitude = msg->altitude;

#ifndef USE_SEMAPHORE
            pthread_mutex_unlock(&metadata.data->lock);
#else
            sem_post(&metadata.data->read_sem);
#endif
            loop++;
        };
        subscription_ = this->create_subscription<gps_msgs::msg::GPSFix>("/nova/navsat", 10, topic_callback);
    }

    ~Subscribernavsat(){
        munmap(navsat_data, sizeof(nv_navsat_data_t));
        close(shm_fd);
    }
};


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscribernavsat>());
    rclcpp::shutdown();
    
    return 0;
}