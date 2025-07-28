#include <iostream>
#include <nova_processing/nv_shm_manager.hpp>

void* producer(void* arg) {
    nv_shm_metadata_t metadata;
    nv_shm_mgr_header header;
    int count = 10;

    snprintf(metadata.shm_name, SHM_NAME_MAX, "test_shm_1\n");
    snprintf(metadata.pname, SHM_NAME_MAX, "test_process_1\n");

    int shm_fd = nvShmManager::nvMapShm(&header);

    nvShmManager::nvGetShmPtr(header, sizeof(int), count, &metadata);
    sem_init(&metadata.data->read_sem, 1, 0);
    sem_init(&metadata.data->write_sem, 1, count);

    int *ptr = (int *)metadata.data->data_ptr;

    for(int i=0;i<100;i++){
        sem_wait(&metadata.data->write_sem);
        ptr[i%count] = 10*i;
        printf("writing %d data \n", i);
        sem_post(&metadata.data->read_sem);
    }

    munmap(header, sizeof(nv_shm_mgr_header_t));
    close(shm_fd);

    return NULL;
}

int main(){
    pthread_t pt1;
    pthread_create(&pt1, NULL, producer, NULL);
    pthread_join(pt1, NULL);


    return 0;
}
