#include <iostream>
#include <nova_processing/nv_shm_manager.hpp>


void signal_handler(int val){
    shm_unlink(SHM_OBJ_NAME);
    exit(0);
}

int main(){

    signal(SIGINT, signal_handler);
    nvShmManager obj(SHM_SIZE_MAX);

    pause();

    return 0;
}
