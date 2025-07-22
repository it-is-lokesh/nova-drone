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
    nv_shm_obj_t shm;
    nv_shm_data shm_data;
    nv_altimeter_data altimeter_data;
public:
    SubscriberAltimeter() : Node("node_subscriber_altimeter"){
        shm.shm_id = SHM_SENSOR_ALTIMETER;
        shm.shm_size = sizeof(nv_shm_data_t) + sizeof(nv_altimeter_data_t);
        snprintf(shm.shm_name, sizeof(shm.shm_name), "/shm_id_%d", shm.shm_id);
        int shm_fd = shm_open(shm.shm_name, O_CREAT | O_RDWR, 0660);
        if(shm_fd == -1) perror("shm_open failed");

        if (ftruncate(shm_fd, shm.shm_size) == -1) perror("ftruncate");

        shm.data = (nv_shm_data)mmap(0, shm.shm_size, PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if(shm.data == MAP_FAILED) perror("mmap failed");

        shm.data->data_ptr = shm.data + sizeof(nv_shm_data_t);
        shm_data = (nv_shm_data)shm.data;
        altimeter_data = (nv_altimeter_data)shm_data->data_ptr;
        printf("shm init done \n");

#ifndef USE_SEMAPHORE
        pthread_mutexattr_init(&shm.attr);
        pthread_mutexattr_setpshared(&shm.attr, PTHREAD_PROCESS_SHARED);
        pthread_mutex_init(&shm.lock, &shm.attr);
        pthread_cond_init(&shm.cond);
        shm.flag = 0;
#else
        printf("before sem init \n");
        sem_init(&shm_data->write_sem, 1, 1);
        printf("init write sem done \n");
        sem_init(&shm_data->read_sem, 1, 0);
#endif

        auto topic_callback = [this](ros_gz_interfaces::msg::Altimeter::UniquePtr msg) -> void {
#ifndef USE_SEMAPHORE
            pthread_mutex_lock(&shm.lock);
#else
            sem_wait(&shm_data->write_sem);
#endif
            altimeter_data->vertical_position = msg->vertical_position;
            altimeter_data->vertical_reference = msg->vertical_reference;
            altimeter_data->vertical_velocity = msg->vertical_velocity;

#ifndef USE_SEMAPHORE
            pthread_mutex_unlock(&shm.lock);
#else
            sem_post(&shm_data->read_sem);
#endif
        };
        subscription_ = this->create_subscription<ros_gz_interfaces::msg::Altimeter>("/nova/altimeter", 10, topic_callback);
    }
};


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberAltimeter>());
    rclcpp::shutdown();
    
    return 0;
}