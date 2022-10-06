#include "comm/comm_stm.h"
#include "comm/pc2stm.h"
#include "comm/stm2pc.h"

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

    recv_timer = NH.createTimer(ros::Duration(0.01), CllbckRecv);
    send_timer = NH.createTimer(ros::Duration(0.01), CllbckSend);

    sub_vel_data = NH.subscribe("/cmd_vel_motor", 16, CllbckVelMotor);

    spinner.spin();

    return 0;
}

void CllbckPc2Stm()
{
    velocity_robot[0] = 10;
    velocity_robot[1] = 20;
    velocity_robot[2] = 30;
    odom_offset[0] = 120;
    odom_offset[1] = 130;
    odom_offset[2] = 200;
    kicker_mode = 0;
    kicker_power = 40;
    kicker_position = 800;
    buzzer_cnt = 10;
    buzzer_time = 10;
}

void CllbckRecv(const ros::TimerEvent &)
{
    // UDP recv
    uint8_t recvlen = recvfrom(fd, recv_buffer, 64, MSG_DONTWAIT, (struct sockaddr *)&stmaddr, &addr_len) + 1;
    if (recvlen > 1 && recv_buffer[0] == 'i' && recv_buffer[1] == 't' && recv_buffer[2] == 's')
    {
        memcpy(&robot_pos[0], recv_buffer + 3, 4);
        memcpy(&robot_pos[1], recv_buffer + 7, 4);
        memcpy(&robot_pos[2], recv_buffer + 11, 4);
        memcpy(&line_sensors, recv_buffer + 15, 1);
        memcpy(&ball_sensors, recv_buffer + 16, 2);
        memcpy(&buttons, recv_buffer + 18, 1);
        memcpy(&ethernet_communication, recv_buffer + 19, 4);
        // comm::stm2pc msg_stm2pc;
        // memcpy(&msg_stm2pc.odom_buffer_x, recv_buffer + 3, 2);
        // memcpy(&msg_stm2pc.odom_buffer_y, recv_buffer + 5, 2);
        // memcpy(&msg_stm2pc.odom_buffer_th, recv_buffer + 7, 2);
        // memcpy(&msg_stm2pc.line_sensors, recv_buffer + 9, 1);
        // memcpy(&msg_stm2pc.gp_sensors, recv_buffer + 10, 2);

        // pub_stm2pc.publish(msg_stm2pc);

        // printf("Pose: %f %f %f | line: %d | ball: %d | Btn: %d | eth: %f \n", robot_pos[0], robot_pos[1], robot_pos[2], line_sensors, ball_sensors, buttons, ethernet_communication);
    }
}
void CllbckSend(const ros::TimerEvent &event)
{
    CllbckPc2Stm();
    //masih eror
    //---Robot Velocity
    //=================
    memcpy(send_buffer + 3, &velocity_robot, 3);
    //---Robot Position
    //=================
    // memcpy(send_buffer + 6, &odom_offset[0], 4);
    // memcpy(send_buffer + 10, &odom_offset[1], 4);
    // memcpy(send_buffer + 14, &odom_offset[2], 4);
    // //---Kicker
    // //=========
    // memcpy(send_buffer + 18, &kicker_mode, 1);
    // memcpy(send_buffer + 19, &kicker_power, 2);
    // memcpy(send_buffer + 21, &buzzer_cnt, 1);
    // memcpy(send_buffer + 22, &buzzer_time, 1);
    // memcpy(send_buffer + 23, &left_dribble_speed, 2);
    // memcpy(send_buffer + 25, &right_dribble_speed, 2);
    // memcpy(send_buffer + 27, &kicker_position, 2);
    // memcpy(send_buffer + 3, &odom_offset[0], 2);
    // memcpy(send_buffer + 5, &odom_offset[1], 2);
    // memcpy(send_buffer + 7, &odom_offset[2], 2);
    // memcpy(send_buffer + 9, &kicker_mode, 1);
    // memcpy(send_buffer + 10, &kicker_power, 1);
    // memcpy(send_buffer + 11, &kicker_position, 2);
    // memcpy(send_buffer + 13, &buzzer_cnt, 1);
    // memcpy(send_buffer + 14, &buzzer_time, 1);

    int8_t cekkkk;
    memcpy(&cekkkk, send_buffer + 4, 1);

    printf("cek: %d \n", cekkkk);

    

    // UDP send
    sendto(fd, (void *)send_buffer, sizeof(send_buffer), 0, (struct sockaddr *)&dst_addr, dst_len);
    printf("send_buffer: %s\n", send_buffer);
}

void CllbckVelMotor(const geometry_msgs::TwistConstPtr &msg)
{
//     printf("Velocity robot: %d %d %d\n", (int)(msg->linear.x), (int)(msg->linear.y), (int)(msg->angular.z));
//     velocity_robot[0] = (int)(msg->linear.x);
//     velocity_robot[1] = (int)(msg->linear.y);
//     velocity_robot[2] = (int)(msg->angular.z);
}