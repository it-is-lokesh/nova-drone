#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/altimeter.hpp>

#include <nova_processing/nv.hpp>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

class SubscriberAltimeter : public rclcpp::Node {
private:
    rclcpp::Subscription<ros_gz_interfaces::msg::Altimeter>::SharedPtr subscription_;
    nv_shm_metadata_t metadata;
    nv_shm_mgr_header header;
    nv_altimeter_data altimeter_data;
    int loop;
    int count;
    int shm_fd;
public:
    SubscriberAltimeter() : Node("node_subscriber_altimeter"){
        count = 10;
        snprintf(metadata.shm_name, SHM_NAME_MAX, "altimeter_shm\n");
        snprintf(metadata.pname, PROCESS_NAME_MAX, "altimeter_interface\n");

        shm_fd = nvShmManager::nvMapShm(&header);

        nvShmManager::nvGetShmPtr(header, sizeof(nv_altimeter_data_t), count, &metadata);

        altimeter_data = (nv_altimeter_data)metadata.data->data_ptr;

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

        auto topic_callback = [this](ros_gz_interfaces::msg::Altimeter::UniquePtr msg) -> void {
#ifndef USE_SEMAPHORE
            pthread_mutex_lock(&metadata.data->lock);
#else
            sem_wait(&metadata.data->write_sem);
#endif
            altimeter_data[loop%count].vertical_position = msg->vertical_position;
            altimeter_data[loop%count].vertical_reference = msg->vertical_reference;
            altimeter_data[loop%count].vertical_velocity = msg->vertical_velocity;

#ifndef USE_SEMAPHORE
            pthread_mutex_unlock(&metadata.data->lock);
#else
            sem_post(&metadata.data->read_sem);
#endif
            loop++;
        };
        subscription_ = this->create_subscription<ros_gz_interfaces::msg::Altimeter>("/nova/altimeter", 10, topic_callback);
    }

    ~SubscriberAltimeter(){
        munmap(altimeter_data, sizeof(nv_altimeter_data_t));
        close(shm_fd);
    }
};


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberAltimeter>());
    rclcpp::shutdown();
    
    return 0;
}