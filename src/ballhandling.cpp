#include <ros/ros.h>
#include "comm/global_var.h"
#include "comm/shmem.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ballhandling");
    return 0;
}