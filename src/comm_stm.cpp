#include <ros/ros.h>
#include "comm/pc2stm.h"
#include "comm/stm2pc.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <arpa/inet.h>
#include <sys/types.h>

#define RECEIVE_PORT 8888
#define SEND_PORT 44444

ros::Timer recv_timer;
ros::Timer send_timer;
ros::Publisher pub_stm2pc;

struct sockaddr_in myaddr;            /* our address */
struct sockaddr_in stmaddr;           /* remote address */
socklen_t addr_len = sizeof(stmaddr); /* length of addresses */
int fd;                               /* our socket */

struct sockaddr_in dst_addr;
socklen_t dst_len = sizeof(dst_addr);

// pc2stm data
int16_t odom_offset_x;
int16_t odom_offset_y;
int16_t odom_offset_th;
uint8_t kicker_mode;
uint8_t kicker_power;
uint16_t kicker_position;
uint8_t buzzer_cnt;
uint8_t buzzer_time;

// stm2pc data
uint8_t buttons;

char recv_buffer[64];
char send_buffer[64] = "its";
char isian[12] = "qweasdz\n";

void cllbck_pc2stm()
{
    // Data percobaan
    odom_offset_x = 12;
    odom_offset_y = 13;
    odom_offset_th = 32;
    kicker_mode = 0;
    kicker_power = 40;
    kicker_position = 800;
    buzzer_cnt = 10;
    buzzer_time = 10;
}

void recv_cllbck(const ros::TimerEvent &)
{
    // UDP recv
    uint8_t recvlen = recvfrom(fd, recv_buffer, 64, MSG_DONTWAIT, (struct sockaddr *)&stmaddr, &addr_len) + 1;
    if (recvlen > 1 && recv_buffer[0] == 'i' && recv_buffer[1] == 't' && recv_buffer[2] == 's')
    {
        // printf("recv: %s\n", recv_buffer);
        comm::stm2pc msg_stm2pc;
        memcpy(&msg_stm2pc.odom_buffer_x, recv_buffer + 3, 2);
        memcpy(&msg_stm2pc.odom_buffer_y, recv_buffer + 5, 2);
        memcpy(&msg_stm2pc.odom_buffer_th, recv_buffer + 7, 2);
        memcpy(&msg_stm2pc.line_sensors, recv_buffer + 9, 1);
        memcpy(&msg_stm2pc.gp_sensors, recv_buffer + 10, 2);

        pub_stm2pc.publish(msg_stm2pc);
    }
}
void send_cllbck(const ros::TimerEvent &)
{
    cllbck_pc2stm();
    memcpy(send_buffer + 3, &odom_offset_x, 2);
    memcpy(send_buffer + 5, &odom_offset_y, 2);
    memcpy(send_buffer + 7, &odom_offset_th, 2);
    memcpy(send_buffer + 9, &kicker_mode, 1);
    memcpy(send_buffer + 10, &kicker_power, 1);
    memcpy(send_buffer + 11, &kicker_position, 2);
    memcpy(send_buffer + 13, &buzzer_cnt, 1);
    memcpy(send_buffer + 14, &buzzer_time, 1);

    // UDP send
    sendto(fd, send_buffer, 15, 0, (struct sockaddr *)&dst_addr, dst_len);
    // printf("send_buffer: %s\n", send_buffer);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_stm");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner spinner(3);

    // Create UDP socket
    if ((fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
    {
        perror("cannot create socket\n");
        return 0;
    }

    // Set PC conf
    bzero(&myaddr, sizeof(struct sockaddr_in));
    myaddr.sin_family = AF_INET;
    myaddr.sin_port = htons(RECEIVE_PORT);
    myaddr.sin_addr.s_addr = INADDR_ANY;

    // Set STM addr
    bzero(&dst_addr, sizeof(struct sockaddr_in));
    dst_addr.sin_family = AF_INET;
    dst_addr.sin_port = htons(RECEIVE_PORT);
    dst_addr.sin_addr.s_addr = inet_addr("169.254.183.100");

    // Bind for recv data from STM
    if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0)
    {
        perror("bind failed");
        return 0;
    }

    // pub_stm2pc = NH.advertise<

    recv_timer = NH.createTimer(ros::Duration(0.01), recv_cllbck);
    send_timer = NH.createTimer(ros::Duration(0.01), send_cllbck);

    ros::Subscriber tes;

    spinner.spin();

    return 0;
}