#ifndef TYPES_H
#define TYPES_H

#include <bits/types.h>

typedef struct shdata_robot_tag
{
    int16_t pos_x;
    int16_t pos_y;
    int16_t pos_z;
    int16_t ball_pos_x;
    int16_t ball_pos_y;
    int16_t state;
} shdata_robot_t;

typedef struct pc2stm_tag
{
    int16_t odom_offset_x;
    int16_t odom_offset_y;
    int16_t odom_offset_theta;
    uint8_t kicker_power;
    uint8_t kicker_mode;
    uint16_t kicker_position;
    uint8_t buzzer_cnt;
    uint8_t buzzer_time;
} pc2stm_t;

typedef struct stm2pc_tag
{
    uint8_t buttons;

} stm2pc_t;

#endif