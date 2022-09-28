/**
 * @brief this node will only work if main program is running correctly
 * */

#include <ros/ros.h>
#include "comm/rs232.h"

#define BAUD_RATE 57600

void cllbck_velocity()
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_motor");
    ros::NodeHandle NH;

    // USB -> 16 - 21
    uint8_t left_motor = 16;
    uint8_t right_motor = 18;
    uint8_t rear_motor = 17;
    char mode[4] = {'8', 'N', '1', 0};
    char control_motor[27] = "IL[3]=4\n;IL[4]=4\n;MO=1\n;";

    if (RS232_OpenComport(left_motor, BAUD_RATE, mode))
    {
        printf("Can not open comport 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        ros::shutdown();
    }
    if (RS232_OpenComport(right_motor, BAUD_RATE, mode))
    {
        printf("Can not open comport 2 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        ros::shutdown();
    }
    if (RS232_OpenComport(rear_motor, BAUD_RATE, mode))
    {
        printf("Can not open comport 3 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        ros::shutdown();
    }

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        static uint8_t prev_motor_state1 = 0;
        unsigned char read_buf1[256];
        uint8_t nrecv1 = RS232_PollComport(rear_motor, read_buf1, sizeof(read_buf1));
        printf("rear: %d %d\n", prev_motor_state1, nrecv1);
        if (prev_motor_state1 == 0x00 && nrecv1 > 0b00)
            RS232_cputs(rear_motor, control_motor);
        prev_motor_state1 = nrecv1;

        static uint8_t prev_motor_state2 = 0;
        unsigned char read_buf2[256];
        uint8_t nrecv2 = RS232_PollComport(left_motor, read_buf2, sizeof(read_buf2));
        printf("left: %d %d\n", prev_motor_state2, nrecv2);
        if (prev_motor_state2 == 0x00 && nrecv2 > 0b00)
            RS232_cputs(left_motor, control_motor);
        prev_motor_state2 = nrecv2;

        static uint8_t prev_motor_state3 = 0;
        unsigned char read_buf3[256];
        uint8_t nrecv3 = RS232_PollComport(right_motor, read_buf3, sizeof(read_buf3));
        printf("right: %d %d\n", prev_motor_state3, nrecv3);
        if (prev_motor_state3 == 0x00 && nrecv3 > 0b00)
            RS232_cputs(right_motor, control_motor);
        prev_motor_state3 = nrecv3;

        RS232_cputs(rear_motor, "MO=1\nJV=0\nBG\nKP[2]=60\nKI[2]=1000\n");
        RS232_cputs(right_motor, "MO=1\nJV=0\nBG\nKP[2]=60\nKI[2]=1000\n");
        RS232_cputs(left_motor, "MO=1\nJV=0\nBG\nKP[2]=60\nKI[2]=1000\n");

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}