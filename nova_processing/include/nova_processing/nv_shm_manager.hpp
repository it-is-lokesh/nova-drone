#ifndef _NV_SHM_MANAGER_HPP_
#define _NV_SHM_MANAGER_HPP_

#include <nova_processing/nv_types.hpp>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <signal.h>
#include <semaphore.h>

#define SHM_SIZE_MAX (2048U*1024U)
#define SHM_COUNT_MAX (10)
#define SHM_NAME_MAX (20)

#define PROCESS_COUNT_MAX (10)
#define PROCESS_NAME_MAX (32)

#define SHM_OBJ_NAME "nv_shm_interface"

/**
 * --------------------------
 * |         HEADER         |
 * --------------------------
 * |     SHM_1_METADATA     |
 * --------------------------
 * |          ...           |
 * --------------------------
 * |     SHM_N_METADATA     |
 * --------------------------
 * |       SHM_1_DATA       |
 * --------------------------
 * |           ...          |
 * --------------------------
 * |       SHM_N_DATA       |
 * -------------------------- 
 */

typedef struct nv_shm_data_t {
    void *data_ptr;
#ifndef USE_SEMAPHORE
    pthread_mutex_t lock;
    pthread_mutexattr_t attr;
    pthread_cond_t cond;
    nv_bool flag;
#else
    sem_t write_sem;
    sem_t read_sem;
#endif
} _nv_shm_data;

typedef _nv_shm_data* nv_shm_data;

typedef struct nv_shm_metadata_t {
    nv_bool taken;
    nv_enum shm_id;
    char shm_name[SHM_NAME_MAX];
    char pname[PROCESS_NAME_MAX];
    nv_size shm_size;
    nv_size data_offset;
    nv_shm_data data;
} _nv_shm_metadata;

typedef _nv_shm_metadata* nv_shm_metadata;

typedef struct nv_shm_mgr_header_t {
    nv_size total_size;
    nv_size used_size;
    nv_size avail_size;
    nv_size shm_offset;
    pthread_mutex_t mutex;
    nv_bool done;
    char pname[PROCESS_COUNT_MAX][PROCESS_NAME_MAX];
    uint64_t pid[PROCESS_COUNT_MAX];
    nv_shm_metadata_t shm_metadata[SHM_COUNT_MAX];
} _nv_shm_mgr_header;

typedef _nv_shm_mgr_header* nv_shm_mgr_header;


class nvShmManager {
public:
    // nv_shm_mgr_header_t header;
    void *data;

    static nv_status nvGetFreeShmId(nv_shm_mgr_header header, uint8_t *shm_id);

    static nv_status nvAdjustOffsets(nv_shm_mgr_header header, nv_size req_size);

    static nv_size nvReserveMem(nv_shm_mgr_header header, nv_size data_size, uint8_t count, char *shm_name);

    static nv_size nvGetMemOffset(nv_shm_mgr_header header, char *name);

    static nv_status nvRegisterProcess(nv_shm_mgr_header header, char *name);

    static nv_status nvGetShmPtr(nv_shm_mgr_header header, nv_size data_size, uint8_t count, nv_shm_metadata metadata);

    static nv_status nvSetShmPtr(nv_shm_mgr_header header, nv_shm_metadata metadata);

    static nv_size nvCalcShmSize(nv_size data_size, uint8_t count);

    static int32_t nvMapShm(nv_shm_mgr_header *header);

public:
    nvShmManager(nv_size shm_size){
        char shm[SHM_NAME_MAX];
        snprintf(shm, sizeof(shm), SHM_OBJ_NAME);
        int shm_fd = shm_open(shm, O_CREAT | O_RDWR, 0660);
        if(shm_fd == -1) perror("shm_open failed \n");

        if(ftruncate(shm_fd, shm_size) == -1) perror("ftruncate failed \n");

        // Initialize header
        nv_shm_mgr_header header;
        header = (nv_shm_mgr_header)mmap(0, sizeof(nv_shm_mgr_header_t), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

        header->total_size = shm_size;
        header->used_size = sizeof(nv_shm_mgr_header_t);
        header->avail_size = shm_size - header->used_size;
        header->shm_offset = header->used_size;

        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);

        pthread_mutex_init(&header->mutex, &attr);
        pthread_mutexattr_destroy(&attr);

        char p_name[PROCESS_NAME_MAX];
        snprintf(p_name, PROCESS_NAME_MAX, "shm_mgr\n");
        nvRegisterProcess(header, p_name);

        header->done = 1;

        munmap(header, sizeof(nv_shm_mgr_header_t));
        close(shm_fd);
    }

    ~nvShmManager(){
        // shm_unlink(SHM_OBJ_NAME);
    }
};


#endif /* _NV_SHM_MANAGER_HPP_ */