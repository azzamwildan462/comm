#include <ros/ros.h>
#include "comm/global_var.h"
#include "comm/shared_mem.h"

void func(ros::Publisher *ppub_stm2pc)
{
    printf("func: %x\n", ppub_stm2pc);
}

void timer()
{
    stm2pc_t stm2pc;
    stm2pc.buttons = 17;
    printf("asd: %d\n", stm2pc.buttons);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_stm");
    ros::NodeHandle NH;

    ros::Publisher pub_stm2pc;

    semShm_t semShmInfo, semShmField;
    semShmInfo.key = 1337;
    semShmField.key = 337;
    int size = 64;
    tes();
    // sem_post(semShmInfo.sem);
    // semShmInfo.segptr = shmOpen(semShmInfo.key, size, &(semShmInfo.shmid), &(semShmInfo.sem), &(semShmInfo.createdSegment));

    // if ((semShmInfo.segptr = shmOpen(semShmInfo.key, size, &(semShmInfo.shmid), &(semShmInfo.sem), &(semShmInfo.createdSegment))) == (void *)-1)
    // {
    //     printf("vis_mu.c:%d | FATAL ERROR, failed to open shared memory segment, exiting.\n Use cmnds ipcs and ipcrm to remove shared memory segment with permissions 666.\nOr reboot.\n", __LINE__);
    //     exit(1);
    // }
    // else
    // {
    //     sem_post(semShmInfo.sem);
    // }

    ros::Rate loop_rate(100);
    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}