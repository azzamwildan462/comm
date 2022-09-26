#include <ros/ros.h>
#include "comm/global_var.h"
#include "comm/shmem.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <arpa/inet.h>
#include <sys/types.h>

#define RECEIVE_PORT 8888
#define SEND_PORT 44444

semShm_t shm_stm2pc; // Shared Memory for STM to PC data
semShm_t shm_pc2stm; // Shared Memory for PC to STM data
char shm_stmpc_data[128];

ros::Timer recv_timer;
ros::Timer send_timer;

struct sockaddr_in myaddr;            /* our address */
struct sockaddr_in stmaddr;           /* remote address */
socklen_t addr_len = sizeof(stmaddr); /* length of addresses */
int fd;                               /* our socket */

void signal_handler(int sig)
{
    if (shm_stm2pc.createdSegment == 1)
        if (shmRemove(shm_stm2pc.shmid, shm_stm2pc.segptr) == -1)
            printf("Unable to detach memory\n");

    printf("[comm_stm] Remove shared memory success\n");
    ros::shutdown();
}

void recv_cllbck(const ros::TimerEvent &)
{
    // UDP recv
    static uint8_t counter = 0;
    uint8_t recvlen = recvfrom(fd, shm_stmpc_data, 64, MSG_DONTWAIT, (struct sockaddr *)&stmaddr, &addr_len);

    // Shared memory works in 50 Hz
    if ((++counter % 2 == 0) || recvlen > 0)
        shmWrite(shm_stmpc_data, 64, shm_stm2pc.shmid, shm_stm2pc.segptr, shm_stm2pc.sem, 1, 0);
}

void send_cllbck(const ros::TimerEvent &)
{
    static uint8_t counter = 0;
    static char shm_pc2stm_data[128];

    // Note urusan logika tendang dan buzzer nanti diurus lagi

    // Shared memory works in 50 Hz
    if (++counter % 2 == 0)
        shmRead(shm_pc2stm_data, 32, shm_pc2stm.shmid, shm_pc2stm.segptr, shm_pc2stm.sem, 0, 0);
    // UDP send
    sendto(fd, shm_pc2stm_data, strlen(shm_pc2stm_data), 0, (struct sockaddr *)&stmaddr, addr_len);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_stm");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner spinner(2);

    signal(SIGINT, signal_handler);

    if (init_shared_mem(&shm_stm2pc, SHM_STM2PC_KEY, 64) == -1)
        ros::shutdown();
    printf("Init SHM_STM2PC SUCCESS\n");
    if (init_shared_mem(&shm_pc2stm, SHM_PC2STM_KEY, 64) == -1)
        ros::shutdown();
    printf("Init SHM_PC2STM SUCCESS\n");

    // Create UDP socket
    if ((fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
    {
        perror("cannot create socket\n");
        return 0;
    }

    // Prepare for binding socket
    bzero(&myaddr, sizeof(struct sockaddr_in));
    myaddr.sin_family = AF_INET;
    myaddr.sin_port = htons(RECEIVE_PORT);
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    // Bind to recv data from STM
    if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0)
    {
        perror("bind failed");
        return 0;
    }

    recv_timer = NH.createTimer(ros::Duration(0.01), recv_cllbck);
    send_timer = NH.createTimer(ros::Duration(0.01), send_cllbck);

    spinner.spin();

    return 0;
}