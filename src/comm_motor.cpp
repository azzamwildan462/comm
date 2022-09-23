#include <ros/ros.h>
#include "comm/global_var.h"
#include "comm/rs232.h"

#define BAUD_RATE 57600

// Shared Memory for subscribing to robot_velocity
semShm_t shm_mc_in;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_motor");
    ros::NodeHandle NH;

    // USB -> 16 - 21
    uint8_t left_motor = 18;
    uint8_t right_motor = 17;
    uint8_t rear_motor = 17;
    char mode[4] = {'8', 'N', '1', 0};
    char control_motor[27] = "IL[3]=4\n;IL[4]=4\n;MO=1\n;";

    // if (RS232_OpenComport(left_motor, BAUD_RATE, mode))
    // {
    //     printf("Can not open comport 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    // }
    // if (RS232_OpenComport(right_motor, BAUD_RATE, mode))
    // {
    //     printf("Can not open comport 2 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    // }
    if (RS232_OpenComport(rear_motor, BAUD_RATE, mode))
    {
        printf("Can not open comport 3 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    }

    // printf("asd: %s\n", control_motor);

    // RS232_cputs(left_motor, control_motor);
    // RS232_cputs(right_motor, control_motor);
    RS232_cputs(rear_motor, control_motor);

    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        // static uint8_t prev_motor_state = 0;
        // unsigned char read_buf[256];
        // uint8_t n = RS232_PollComport(rear_motor, read_buf, sizeof(read_buf));
        // printf("%d %d %s\n", prev_motor_state, n, read_buf);
        // if (prev_motor_state == 0x00 && n == 0x08)
        //     RS232_cputs(rear_motor, control_motor);
        // prev_motor_state = n;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}