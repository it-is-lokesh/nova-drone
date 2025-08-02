#include <nova_processing/nv.hpp>

nv_status nvShmManager::nvRegisterProcess(nv_shm_mgr_header header, char *name){
    int i;
    for(i=0;i<PROCESS_COUNT_MAX;i++){
        if(header->pid[i] == 0){
            memcpy(header->pname[i], name, PROCESS_NAME_MAX);
            header->pid[i] = getpid();
            break;
        }
    }

    return 0;
}

nv_size nvShmManager::nvGetMemOffset(nv_shm_mgr_header header, char *name){

    nv_shm_metadata metadata;
    nv_size offset;

    for(int i = 0; i < SHM_COUNT_MAX; i++){
        metadata = &header->shm_metadata[i];
        if(strcmp(metadata->shm_name, name) == 0){
            offset = metadata->data_offset;
            break;
        }
    }

    return offset;
}

nv_size nvShmManager::nvReserveMem(nv_shm_mgr_header header, nv_size data_size, uint8_t count, char *shm_name){
    pthread_mutex_lock(&header->mutex);

    nv_size req_size = nvCalcShmSize(data_size, count);

    if(header->avail_size < req_size) return 0xFFFFFFFF;

    nv_size offset = header->shm_offset;

    nvAdjustOffsets(header, req_size);

    uint8_t shm_id;

    nvGetFreeShmId(header, &shm_id);

    memcpy(header->shm_metadata->shm_name, shm_name, SHM_NAME_MAX);
    header->shm_metadata->shm_id = shm_id;
    header->shm_metadata->shm_size = req_size;
    header->shm_metadata->data_offset = offset;

    pthread_mutex_unlock(&header->mutex);

    return offset;
}

nv_status nvShmManager::nvAdjustOffsets(nv_shm_mgr_header header, nv_size req_size){
    header->shm_offset += req_size;
    header->used_size += req_size;
    header->avail_size -= req_size;

    return 0;
}

nv_status nvShmManager::nvGetFreeShmId(nv_shm_mgr_header header, uint8_t *shm_id){
    int8_t shm_id_itr = -1;

    nv_shm_metadata metadata;
    for(shm_id_itr = 0; shm_id_itr < SHM_COUNT_MAX; shm_id_itr++){
        metadata = &header->shm_metadata[shm_id_itr];
        if(metadata->taken == false){
            metadata->taken = true;
            *shm_id = shm_id_itr;
            break;
        }
    }

    return 0;
}

nv_status nvShmManager::nvGetShmPtr(nv_shm_mgr_header header, nv_size data_size, uint8_t count, nv_shm_metadata metadata){

    nv_size offset = nvReserveMem(header, data_size, count, metadata->shm_name);

    header->shm_metadata->data = (nv_shm_data)((char *)header + offset);
    header->shm_metadata->data->data_ptr = (char *)header->shm_metadata->data + sizeof(nv_shm_data_t);

    metadata->data = header->shm_metadata->data;
    metadata->data->data_ptr = header->shm_metadata->data->data_ptr;

    nvRegisterProcess(header, metadata->pname);

    return 0;
}

nv_size nvShmManager::nvCalcShmSize(nv_size data_size, uint8_t count){
    nv_size size;
    size = sizeof(nv_shm_data_t) + count * data_size;
    return size;
}

nv_status nvShmManager::nvSetShmPtr(nv_shm_mgr_header header, nv_shm_metadata metadata){

    nv_size offset = nvGetMemOffset(header, metadata->shm_name);

    header->shm_metadata->data = (nv_shm_data)((char *)header + offset);
    header->shm_metadata->data->data_ptr = (char *)header->shm_metadata->data + sizeof(nv_shm_data_t);

    metadata->data = header->shm_metadata->data;
    metadata->data->data_ptr = header->shm_metadata->data->data_ptr;

    nvRegisterProcess(header, metadata->pname);

    return 0;
}

int32_t nvShmManager::nvMapShm(nv_shm_mgr_header *header){
    char shm[SHM_NAME_MAX];
    snprintf(shm, sizeof(shm), SHM_OBJ_NAME);
    int shm_fd = shm_open(shm, O_CREAT | O_RDWR, 0660);
    if(shm_fd == -1) perror("shm_open failed \n");

    *header = (nv_shm_mgr_header)mmap(0, SHM_SIZE_MAX, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

    return shm_fd;
}
