#include <iostream>
#include <nova_processing/nv_shm_manager.hpp>

void* consumer(void* arg) {
    nv_shm_metadata_t metadata;
    nv_shm_mgr_header header;
    int count = 10;

    snprintf(metadata.shm_name, SHM_NAME_MAX, "test_shm_1\n");
    snprintf(metadata.pname, SHM_NAME_MAX, "test_process_2\n");

    int shm_fd = nvShmManager::nvMapShm(&header);

    nvShmManager::nvSetShmPtr(header, &metadata);

    int *ptr = (int *)metadata.data->data_ptr;

    for(int i=0;i<100;i++){
        sem_wait(&metadata.data->read_sem);
        printf("data: %d \n", ptr[i%count]);
        sem_post(&metadata.data->write_sem);
    }

    munmap(header, sizeof(nv_shm_mgr_header_t));
    close(shm_fd);

    return NULL;
}

int main(){
    pthread_t pt2;
    pthread_create(&pt2, NULL, consumer, NULL);
    pthread_join(pt2, NULL);


    return 0;
}
