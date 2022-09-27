#include <ros/ros.h>
#include "comm/global_var.h"
#include "comm/shmem.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <arpa/inet.h>
#include <sys/types.h>

#define RECEIVE_PORT 8888
#define SEND_PORT 44444

ros::Timer recv_timer;
ros::Timer send_timer;

struct sockaddr_in myaddr;            /* our address */
struct sockaddr_in stmaddr;           /* remote address */
socklen_t addr_len = sizeof(stmaddr); /* length of addresses */
int fd;                               /* our socket */

char recv_buffer[64];
char send_buffer[64];

void recv_cllbck(const ros::TimerEvent &)
{
    // UDP recv
    uint8_t recvlen = recvfrom(fd, recv_buffer, 64, MSG_DONTWAIT, (struct sockaddr *)&stmaddr, &addr_len);
}
void send_cllbck(const ros::TimerEvent &)
{
    // UDP send
    sendto(fd, send_buffer, strlen(send_buffer), 0, (struct sockaddr *)&stmaddr, addr_len);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_stm");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner spinner(2);

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