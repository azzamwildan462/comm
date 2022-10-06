#ifndef COMM_STM_H_
#define COMM_STM_H_
#include <ros/ros.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <arpa/inet.h>
#include <sys/types.h>

#include "geometry_msgs/Twist.h"

#define RECEIVE_PORT 8888
#define SEND_PORT 44444

//---Timer
//========
ros::Timer recv_timer;
ros::Timer send_timer;

//---Publisher
//============
ros::Publisher pub_stm2pc;

//---Subscriber
//=============
ros::Subscriber sub_vel_data;

struct sockaddr_in myaddr;            /* our address */
struct sockaddr_in stmaddr;           /* remote address */
socklen_t addr_len = sizeof(stmaddr); /* length of addresses */
int fd;                               /* our socket */

struct sockaddr_in dst_addr;
socklen_t dst_len = sizeof(dst_addr);

//============
// pc2stm data
//============
/**
 * @param [0]:  vx
 * @param [1]: vy
 * @param [2]: vth
 */
int8_t velocity_robot[3];
short int vel_x;
short int vel_y;
short int vel_th;
/**
 * @param [0]: x
 * @param [1]: y
 * @param [2]: th
 */
int16_t odom_offset[3];

uint8_t kicker_mode;
uint16_t kicker_power;

uint8_t buzzer_cnt;
uint8_t buzzer_time;

int16_t left_dribble_speed;
int16_t right_dribble_speed;

uint16_t kicker_position;

uint8_t color_status;

//============
// pc2stm data
//============

// stm2pc data
uint8_t buttons;
uint8_t line_sensors;
uint16_t ball_sensors;
float robot_pos[3];
float ethernet_communication;

char recv_buffer[64];
char send_buffer[64] = "its";
// char send_buffer[64] = {'i', 't', 's'};
char isian[12] = "qweasdz\n";

//---Prototypes
void CllbckPc2Stm();
void CllbckRecv(const ros::TimerEvent &);
void CllbckSend(const ros::TimerEvent &);
void CllbckVelMotor(const geometry_msgs::TwistConstPtr &msg);

#endif