#include "ros/ros.h"
#include "ros/package.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <arpa/inet.h>
#include <sys/types.h>

#include "yaml-cpp/yaml.h"
#include <chrono>

#define USE_KEYDB

#ifdef USE_KEYDB
#include <redis-cpp/stream.h>
#include <redis-cpp/execute.h>
#endif

struct sockaddr_in myaddr;
struct sockaddr_in bsaddr;
socklen_t addr_len = sizeof(bsaddr);
int fd;

struct sockaddr_in dst_addr;
socklen_t dst_len = sizeof(dst_addr);

// Cfg
uint16_t my_port;
uint16_t BS_port;
char BS_addr[16];

// Ros-utils
ros::Timer send_timer;
ros::Timer recv_timer;

// UDP data
char send_buf[64];
uint8_t actual_data_size;
char its[4] = "its";

void CllbckRecv(const ros::TimerEvent &event)
{
    // printf("recv\n");
    static char recv_buf[128];
    uint8_t nrecv = recvfrom(fd, recv_buf, 64, MSG_DONTWAIT, (struct sockaddr *)&bsaddr, &addr_len);
}

void CllbckSend(const ros::TimerEvent &event)
{
    // printf("send..\n");
    actual_data_size = 3;
    memcpy(send_buf, its, 3);
    uint8_t nsent = sendto(fd, send_buf, actual_data_size, 0, (struct sockaddr *)&dst_addr, sizeof(struct sockaddr));
}

void loadConfig()
{
    char *robot_num = getenv("ROBOT");
    char config_file[100];
    std::string current_dir = ros::package::getPath("comm");
    sprintf(config_file, "%s/../../config/IRIS%s/unicast.yaml", current_dir.c_str(), robot_num);
    printf("config file: %s\n", config_file);

    YAML::Node config = YAML::LoadFile(config_file);
    strcpy(BS_addr, config["BS_addr"].as<std::string>().c_str());
    BS_port = config["BS_port"].as<int>();
    my_port = config["my_port"].as<int>();

    printf("BS: %s | %d\n", BS_addr, BS_port);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_unicast");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner spinner(3);

    loadConfig();

    // Create UDP socket
    if ((fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
    {
        perror("cannot create socket\n");
        return 0;
    }

    // Set PC conf
    bzero(&myaddr, sizeof(struct sockaddr_in));
    myaddr.sin_family = AF_INET;
    myaddr.sin_port = htons(my_port);
    myaddr.sin_addr.s_addr = INADDR_ANY;

    // Set BS addr conf
    bzero(&dst_addr, sizeof(struct sockaddr_in));
    dst_addr.sin_family = AF_INET;
    dst_addr.sin_port = htons(BS_port);
    dst_addr.sin_addr.s_addr = inet_addr(BS_addr);

    // Bind for recv data from BS
    if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0)
    {
        perror("bind failed");
        return 0;
    }

    recv_timer = NH.createTimer(ros::Duration(0.01), CllbckRecv);
    send_timer = NH.createTimer(ros::Duration(0.01), CllbckSend);

    spinner.spin();

    return 0;
}