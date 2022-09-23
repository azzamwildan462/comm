#include <ros/ros.h>
#include "comm/global_var.h"
#include "comm/shmem.h"

// Shared memory for Incoming / Outcoming multicast data
semShm_t shm_mc_in;
semShm_t shm_mc_out;
char shm_mc_in_data[64];
char shm_mc_out_data[64];

ros::Timer recv_timer;
ros::Timer send_timer;

void signal_handler(int sig)
{
    if (shm_mc_in.createdSegment == 1)
        if (shmRemove(shm_mc_in.shmid, shm_mc_in.segptr) == -1)
            printf("Unable to detach memory\n");
    if (shm_mc_out.createdSegment == 1)
        if (shmRemove(shm_mc_out.shmid, shm_mc_out.segptr) == -1)
            printf("Unable to detach memory\n");
    printf("Remove success\n");
    ros::shutdown();
}

void recv_cllbck(const ros::TimerEvent &)
{
}

void send_cllbck(const ros::TimerEvent &)
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_multicast");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner spinner(0);

    signal(SIGINT, signal_handler);

    if (init_shared_mem(&shm_mc_in, SHM_STM2PC_KEY, 64) == -1)
        ros::shutdown();
    if (init_shared_mem(&shm_mc_out, SHM_MC_OUT_KEY, 64) == -1)
        ros::shutdown();
    uint16_t coba;
    uint16_t coba2;
    uint16_t coba3;

    recv_timer = NH.createTimer(ros::Duration(0.1), recv_cllbck);
    send_timer = NH.createTimer(ros::Duration(0.1), send_cllbck);

    // ros::Rate loop_rate(20);
    // while (ros::ok())
    // {
    //     shmRead(shm_mc_in_data, 6, shm_mc_in.shmid, shm_mc_in.segptr, shm_mc_in.sem, 1, 0);
    //     memcpy(&coba, shm_mc_in_data, 2);
    //     memcpy(&coba2, shm_mc_in_data + 2, 2);
    //     memcpy(&coba3, shm_mc_in_data + 4, 2);
    //     printf("read: %d %d %d\n", coba, coba2, coba3);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    spinner.spin();

    return 0;
}