#ifndef TYPES_H
#define TYPES_H

#include <bits/types.h>
#include <signal.h>
#include <unistd.h>
#include "comm/shmem.h"

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

// Shared Memory Flags
int8_t SHM_STM2PC_KEY = 0x01;
int8_t SHM_PC2STM_KEY = 0x02;
int8_t SHM_MC_IN_KEY = 0x04;
int8_t SHM_MC_OUT_KEY = 0x10;

#endif