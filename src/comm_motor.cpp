/**
 * @brief this node will only work if main program is running correctly
 * */

#include <ros/ros.h>
#include "comm/rs232.h"
#include "geometry_msgs/Twist.h"

#define BAUD_RATE 57600
#define DEG2RAD 0.017452925
#define RAD2DEG 57.295780

#define DEFAULT_MOTOR_GAIN 20
#define LEFT_MOTOR_GAIN DEFAULT_MOTOR_GAIN
#define RIGHT_MOTOR_GAIN DEFAULT_MOTOR_GAIN + 2
#define REAR_MOTOR_GAIN DEFAULT_MOTOR_GAIN

#define DEFAULT_DRIBBLE_GAIN 40
#define LEFT_DRIBBLE_GAIN DEFAULT_DRIBBLE_GAIN
#define RIGHT_DRIBBLE_GAIN DEFAULT_DRIBBLE_GAIN

ros::Timer tim_control_motor;
ros::Timer tim_control_dribble;

// USB -> 16 - 21
uint8_t left_motor = 17;
uint8_t right_motor = 16;
uint8_t rear_motor = 18;

float vel_x = 0;
float vel_y = 0;
float vel_th = 0;
float accel;
float deccel;

char control_cmd[32] = "IL[3]=4\n;IL[4]=4\n;MO=1\n;";

ros::Subscriber sub_vel_motor;

void CllbckVelMotor(const geometry_msgs::TwistConstPtr &msg);

// Untuk test
uint8_t kbhit()
{
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized)
    {
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

void cllbck_velocity()
{
    // Hardcode

    if (kbhit())
    {
        char key = std::cin.get();

        switch (key)
        {
        case 'j':
            vel_x = 0;
            vel_y = 10;
            vel_th = 0;
            accel = 2;
            deccel = accel;
            break;
        case 'n':
            vel_x = 0;
            vel_y = -10;
            vel_th = 0;
            accel = 2;
            deccel = accel;
            break;
        case ' ':
            vel_x = 0;
            vel_y = 0;
            vel_th = 0;
            accel = 2;
            deccel = accel;
            break;
        }
    }
}

void control_motor(const ros::TimerEvent &)
{
    // 1 aja? apa semua?
    static uint8_t prev_motor_state1 = 0;
    unsigned char read_buf1[256];
    uint8_t nrecv1 = RS232_PollComport(rear_motor, read_buf1, sizeof(read_buf1));
    if (prev_motor_state1 == 0x00 && nrecv1 > 0b00)
        RS232_cputs(rear_motor, control_cmd);
    prev_motor_state1 = nrecv1;

    static uint8_t prev_motor_state2 = 0;
    unsigned char read_buf2[256];
    uint8_t nrecv2 = RS232_PollComport(left_motor, read_buf2, sizeof(read_buf2));
    if (prev_motor_state2 == 0x00 && nrecv2 > 0b00)
        RS232_cputs(left_motor, control_cmd);
    prev_motor_state2 = nrecv2;

    static uint8_t prev_motor_state3 = 0;
    unsigned char read_buf3[256];
    uint8_t nrecv3 = RS232_PollComport(right_motor, read_buf3, sizeof(read_buf3));
    if (prev_motor_state3 == 0x00 && nrecv3 > 0b00)
        RS232_cputs(right_motor, control_cmd);
    prev_motor_state3 = nrecv3;

    // cllbck_velocity();
    static float vel_left_motor;
    static float vel_right_motor;
    static float vel_rear_motor;

    vel_left_motor = vel_th - vel_x * sinf(30 * DEG2RAD) - vel_y * cosf(30 * DEG2RAD);
    vel_right_motor = vel_th - vel_x * sinf(30 * DEG2RAD) + vel_y * cosf(30 * DEG2RAD);
    vel_rear_motor = vel_th + vel_x;

    vel_left_motor *= 66.6666 * LEFT_MOTOR_GAIN;
    vel_right_motor *= 66.6666 * RIGHT_MOTOR_GAIN;
    vel_rear_motor *= 66.6666 * REAR_MOTOR_GAIN;

    // Hardcode ngontrol 0 sementara..
    // RS232_cputs(rear_motor, "MO=1\nJV=0\nBG\nKP[2]=60\nKI[2]=1000\n");
    // RS232_cputs(right_motor, "MO=1\nJV=0\nBG\nKP[2]=60\nKI[2]=1000\n");
    // RS232_cputs(left_motor, "MO=1\nJV=0\nBG\nKP[2]=60\nKI[2]=1000\n");

    char cmd_mtr_left[40];
    char cmd_mtr_right[40];
    char cmd_mtr_rear[40];

    sprintf(cmd_mtr_left, "AC=3\nJV=%d\nBG\nKP[2]=60\nKI[2]=1000\n", (int)(vel_left_motor));
    sprintf(cmd_mtr_right, "AC=3\nJV=%d\nBG\nKP[2]=60\nKI[2]=1000\n", (int)(vel_right_motor));
    sprintf(cmd_mtr_rear, "AC=3\nJV=%d\nBG\nKP[2]=60\nKI[2]=1000\n", (int)(vel_rear_motor));

    RS232_cputs(rear_motor, cmd_mtr_left);
    RS232_cputs(right_motor, cmd_mtr_right);
    RS232_cputs(left_motor, cmd_mtr_rear);
}

void control_dribble(const ros::TimerEvent &)
{
    cllbck_velocity();

    static float left_dribble_vel;
    static float right_dribble_vel;

    left_dribble_vel = (vel_th - vel_x * sinf(30 * DEG2RAD) - vel_y * cosf(30 * DEG2RAD)) + (vel_th + vel_x * cos(DEG2RAD * 120));
    right_dribble_vel = ((vel_th - vel_x * sinf(30 * DEG2RAD) + vel_y * cosf(30 * DEG2RAD) * -1)) + (vel_th + vel_x * cos(DEG2RAD * 60));

    left_dribble_vel *= LEFT_DRIBBLE_GAIN;
    right_dribble_vel *= RIGHT_DRIBBLE_GAIN;

    // printf("dbg: %.02f %.02f\n", left_dribble_vel, right_dribble_vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_motor");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner spinner(3);

    char mode[4] = {'8', 'N', '1', 0};

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

    tim_control_motor = NH.createTimer(ros::Duration(0.01), control_motor);
    tim_control_dribble = NH.createTimer(ros::Duration(0.01), control_dribble);

    sub_vel_motor = NH.subscribe("/cmd_vel_motor", 16, CllbckVelMotor);

    spinner.spin();

    return 0;
}

void CllbckVelMotor(const geometry_msgs::TwistConstPtr &msg)
{
    // printf("data dai cllbck: %.02f %.02f %.02f\n", msg->linear.x, msg->linear.y, msg->angular.z);
    vel_x = msg->linear.x;
    vel_y = msg->linear.y;
    vel_th = msg->angular.z;
}