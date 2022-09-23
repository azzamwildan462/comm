#include <ros/ros.h>
#include "comm/global_var.h"
#include "comm/shmem.h"

semShm_t shm_stm2pc; // Shared Memory for STM to PC data
semShm_t shm_pc2stm; // Shared Memory for PC to STM data

void signal_handler(int sig)
{
    if (shm_stm2pc.createdSegment == 1)
        if (shmRemove(shm_stm2pc.shmid, shm_stm2pc.segptr) == -1)
            printf("Unable to detach memory\n");

    printf("Remove success\n");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_stm");
    ros::NodeHandle NH;

    signal(SIGINT, signal_handler);

    if (init_shared_mem(&shm_stm2pc, SHM_STM2PC_KEY, 64) == -1)
        ros::shutdown();
    printf("Init SHM_STM2PC SUCCESS\n");

    char shm_stmpc_data[128];
    uint16_t test = 90;
    uint16_t test2 = 180;
    uint16_t test3 = 200;

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        test++;
        test2++;
        test3++;
        memcpy(shm_stmpc_data, &test, 2);
        memcpy(shm_stmpc_data + 2, &test2, 2);
        memcpy(shm_stmpc_data + 4, &test3, 2);
        shmWrite(shm_stmpc_data, 6, shm_stm2pc.shmid, shm_stm2pc.segptr, shm_stm2pc.sem, 1, 0);
        // printf("write: %d %d %d\n", test, test2, test3);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}