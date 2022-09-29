#include <ros/ros.h>
#include <ros/package.h>
#include "comm/global_var.h"
#include "comm/mc_in.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <fstream>
#include <sys/time.h>
#include <sched.h>
#include <unistd.h>
#include <sstream>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <unistd.h>
#include <linux/if_ether.h>

#include "yaml-cpp/yaml.h"
#include <chrono>

#define USE_KEYDB

#ifdef USE_KEYDB
#include <redis-cpp/stream.h>
#include <redis-cpp/execute.h>
#endif

#define PERR(txt, par...) \
    printf("ERROR: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ##par)
#define PERRNO(txt) \
    printf("ERROR: (%s / %s): " txt ": %s\n", __FILE__, __FUNCTION__, strerror(errno))

// Socket
typedef struct multiSocket_tag
{
    struct sockaddr_in destAddress;
    int socketID;
    bool compressedData;
} multiSocket_t;
typedef struct nw_config
{
    char multicast_ip[16];
    char iface[10];
    char identifier[1];
    unsigned int port;
    uint8_t compress_type;
} config;
multiSocket_t multiSocket;
multiSocket_t *recv_socket;
struct sockaddr src_addr;
socklen_t addr_len = sizeof(src_addr);

// Config
config nw_config;

ros::Timer recv_timer;
ros::Timer send_timer;

ros::Publisher pub_bs2pc;
ros::Subscriber sub_pc2mc;

// PC2MC_out datas
int16_t pos_x;
int16_t pos_y;
int16_t theta;
int8_t ball_status;
int16_t ball_x;
int16_t ball_y;
uint16_t robot_cond;
int8_t pass_target;

// UDP data
char send_buf[64];
uint8_t actual_data_size;
char its[4] = "its";

// KeyDB
auto keydb_put_stream = rediscpp::make_stream("127.0.0.1", "6969");
uint8_t delay_tolerance = 10;
int64_t robot_epoch[6];

// Titip
// redis-cli -p 6969 SET data_valid1 0
// redis-cli -p 6969 SET data_valid2 0
// redis-cli -p 6969 SET data_valid3 0
// redis-cli -p 6969 SET data_valid4 0
// redis-cli -p 6969 SET data_valid5 0

int if_NameToIndex(char *ifname, char *address)
{
    int fd;
    struct ifreq if_info;
    int if_index;

    memset(&if_info, 0, sizeof(if_info));
    strncpy(if_info.ifr_name, ifname, IFNAMSIZ - 1);

    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        PERRNO("socket");
        return -1;
    }
    if (ioctl(fd, SIOCGIFINDEX, &if_info) == -1)
    {
        PERRNO("ioctl");
        close(fd);
        return -1;
    }
    if_index = if_info.ifr_ifindex;

    if (ioctl(fd, SIOCGIFADDR, &if_info) == -1)
    {
        PERRNO("ioctl");
        close(fd);
        return -1;
    }

    close(fd);

    sprintf(address, "%d.%d.%d.%d\n",
            (int)((unsigned char *)if_info.ifr_hwaddr.sa_data)[2],
            (int)((unsigned char *)if_info.ifr_hwaddr.sa_data)[3],
            (int)((unsigned char *)if_info.ifr_hwaddr.sa_data)[4],
            (int)((unsigned char *)if_info.ifr_hwaddr.sa_data)[5]);
#ifdef COMM_DEBUG
    printf("**** Using device %s -> %s\n", if_info.ifr_name, address);
#endif

    return if_index;
}
int openSocket()
{
    struct sockaddr_in multicastAddress;
    struct ip_mreqn mreqn;
    struct ip_mreq mreq;
    int opt;
    char address[16]; // IPV4: xxx.xxx.xxx.xxx/0

    bzero(&multicastAddress, sizeof(struct sockaddr_in));
    multicastAddress.sin_family = AF_INET;
    multicastAddress.sin_port = htons(nw_config.port);
    multicastAddress.sin_addr.s_addr = INADDR_ANY;

    bzero(&multiSocket.destAddress, sizeof(struct sockaddr_in));
    multiSocket.destAddress.sin_family = AF_INET;
    multiSocket.destAddress.sin_port = htons(nw_config.port);
    multiSocket.destAddress.sin_addr.s_addr = inet_addr(nw_config.multicast_ip);

    if ((multiSocket.socketID = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        PERRNO("socket");
        return -1;
    }

    memset((void *)&mreqn, 0, sizeof(mreqn));
    mreqn.imr_ifindex = if_NameToIndex(nw_config.iface, address);
    if ((setsockopt(multiSocket.socketID, SOL_IP, IP_MULTICAST_IF, &mreqn, sizeof(mreqn))) == -1)
    {
        PERRNO("setsockopt 1");
        return -1;
    }

    opt = 1;
    if ((setsockopt(multiSocket.socketID, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) == -1)
    {
        PERRNO("setsockopt 2");
        return -1;
    }

    memset((void *)&mreq, 0, sizeof(mreq));
    mreq.imr_multiaddr.s_addr = inet_addr(nw_config.multicast_ip);
    mreq.imr_interface.s_addr = inet_addr(address);

    if ((setsockopt(multiSocket.socketID, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq))) == -1)
    {
        PERRNO("setsockopt 3");
        printf("\nerrno %d\n", errno);
        return -1;
    }

    /* 0 for Disable reception of our own multicast */
    opt = 1;
    if ((setsockopt(multiSocket.socketID, IPPROTO_IP, IP_MULTICAST_LOOP, &opt, sizeof(opt))) == -1)
    {
        PERRNO("setsockopt");
        return -1;
    }

    if (bind(multiSocket.socketID, (struct sockaddr *)&multicastAddress, sizeof(struct sockaddr_in)) == -1)
    {
        PERRNO("bind");
        return -1;
    }

    return 0;
}
void closeSocket()
{
    if (multiSocket.socketID != -1)
        shutdown(multiSocket.socketID, SHUT_RDWR);
}

void loadConfig()
{
    // printf("Loading configuration..\n");
    char *robot_num = getenv("ROBOT");
    char config_file[100];
    std::string current_dir = ros::package::getPath("comm");
    sprintf(config_file, "%s/../../config/IRIS%s/multicast.yaml", current_dir.c_str(), robot_num);
    printf("config file: %s\n", config_file);

    YAML::Node config = YAML::LoadFile(config_file);
    strcpy(nw_config.identifier, config["identifier"].as<std::string>().c_str());
    strcpy(nw_config.iface, config["iface"].as<std::string>().c_str());
    strcpy(nw_config.multicast_ip, config["multicast_ip"].as<std::string>().c_str());

    nw_config.port = config["port"].as<int>();

    printf("identifier: %s\n", nw_config.identifier);
    printf("iface: %s\n", nw_config.iface);
    printf("multicast_ip: %s\n", nw_config.multicast_ip);
    printf("port: %d\n", nw_config.port);
}

void updateRobotsStatus()
{
    int64_t epoch_now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    uint8_t status_valid = 0;
    char s_valid_key[16] = "data_valid";
    for (uint8_t i = 1; i <= 5; i++)
    {
        uint8_t id = i + '0';
        memcpy(s_valid_key + 10, &id, 1);
        status_valid = (abs(epoch_now - robot_epoch[i]) <= delay_tolerance);
        auto const put_value = std::to_string(status_valid);
        rediscpp::execute_no_flush(*keydb_put_stream, "set", s_valid_key, put_value);
        // printf("%s: %d - %d -> %d (%d)\n", s_valid_key, epoch_now, robot_epoch[i], abs(epoch_now - robot_epoch[i]), status_valid);
    }

    std::flush(*keydb_put_stream);
}

void putRobotsData(char data[])
{
    uint16_t val_buffer_2B;
    uint8_t val_buffer_1B;
    char put_keys[9][12] = {"pos_x", "pos_y", "theta", "ball_stat", "ball_x", "ball_y", "r_cond", "pass_target"};
    uint8_t put_keys_size[9] = {6, 6, 6, 10, 7, 7, 7, 12};
    uint8_t val_pos[9] = {12, 14, 16, 18, 19, 21, 23, 25};
    uint8_t val_size[9] = {2, 2, 2, 1, 2, 2, 2, 1};

    // Update robots epoch
    int64_t recv_epoch;
    uint8_t recv_from;
    memcpy(&recv_from, data, 1);
    memcpy(&recv_epoch, data + 1, 8);
    recv_from -= '0';
    robot_epoch[recv_from] = recv_epoch;

    for (uint8_t i = 0; i < 8; i++)
    {
        bzero(&val_buffer_2B, 2);
        memcpy(&val_buffer_2B, data + val_pos[i], val_size[i]);
        memcpy(put_keys[i] + put_keys_size[i] - 1, data, 1);
        auto const put_value = std::to_string(val_buffer_2B);
        printf("%s: %s\n", put_keys[i], put_value.c_str());
        rediscpp::execute_no_flush(*keydb_put_stream, "set", put_keys[i], put_value);
    }

    std::flush(*keydb_put_stream);
}

void cllbck_pc2mc()
{
    // Data sintetis buat nyoba
    pos_x = 123;
    pos_y = 1000;
    theta = 69;
    ball_status = 1;
    ball_x = 300;
    ball_y = 700;
    robot_cond = 10;
    pass_target = 0;

    int64_t epoch = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    memcpy(send_buf, its, 3);
    memcpy(send_buf + 3, nw_config.identifier, 1);
    memcpy(send_buf + 4, &epoch, 8);
    memcpy(send_buf + 12, &pos_x, 2);
    memcpy(send_buf + 14, &pos_y, 2);
    memcpy(send_buf + 16, &theta, 2);
    memcpy(send_buf + 18, &ball_status, 1);
    memcpy(send_buf + 19, &ball_x, 2);
    memcpy(send_buf + 21, &ball_y, 2);
    memcpy(send_buf + 23, &robot_cond, 2);
    memcpy(send_buf + 25, &pass_target, 1);

    actual_data_size = 26;
}

void recv_cllbck(const ros::TimerEvent &)
{
    char recv_buf[64];
    uint8_t nrecv = recvfrom(recv_socket->socketID, recv_buf, 64, MSG_DONTWAIT, &src_addr, &addr_len);
    if (nrecv < 255 && (recv_buf[3] >= '0' && recv_buf[3] <= '5') && recv_buf[0] == 'i')
    {
        uint8_t identifier = recv_buf[3] - '0';

        // Recv from Basestation
        if (identifier == 0)
        {
            comm::mc_in msg_mc_to_pc;
            printf("BS: %s\n", recv_buf);
            memcpy(&msg_mc_to_pc.base, recv_buf + 4, 1);
            memcpy(&msg_mc_to_pc.command, recv_buf + 5, 1);
            memcpy(&msg_mc_to_pc.style, recv_buf + 6, 1);
            memcpy(&msg_mc_to_pc.ball_x_field, recv_buf + 7, 2);
            memcpy(&msg_mc_to_pc.ball_y_field, recv_buf + 9, 2);
            memcpy(&msg_mc_to_pc.manual_x, recv_buf + 11, 2);
            memcpy(&msg_mc_to_pc.manual_y, recv_buf + 13, 2);
            memcpy(&msg_mc_to_pc.manual_th, recv_buf + 15, 2);
            memcpy(&msg_mc_to_pc.offset_x, recv_buf + 17, 2);
            memcpy(&msg_mc_to_pc.offset_y, recv_buf + 19, 2);
            memcpy(&msg_mc_to_pc.offset_th, recv_buf + 21, 2);
            memcpy(&msg_mc_to_pc.data_mux1, recv_buf + 23, 2);
            memcpy(&msg_mc_to_pc.data_mux2, recv_buf + 25, 2);
            memcpy(&msg_mc_to_pc.mux_control, recv_buf + 27, 2);
            memcpy(&msg_mc_to_pc.trans_vel_trim, recv_buf + 29 + atoi(nw_config.identifier) - 1, 1);
            memcpy(&msg_mc_to_pc.rotary_vel_trim, recv_buf + 34 + atoi(nw_config.identifier) - 1, 1);
            memcpy(&msg_mc_to_pc.kick_power_trim, recv_buf + 39 + atoi(nw_config.identifier) - 1, 1);

            pub_bs2pc.publish(msg_mc_to_pc);
        }
        else
        {
#ifdef USE_KEYDB
            // putRobotsData(recv_buf);
#else

#endif
        }
    }
}

void send_cllbck(const ros::TimerEvent &)
{
    cllbck_pc2mc();
    uint8_t nsent = sendto(multiSocket.socketID, send_buf, actual_data_size, 0, (struct sockaddr *)&multiSocket.destAddress, sizeof(struct sockaddr));
    if (nsent == actual_data_size)
    {
        // printf("send: %s | size %d\n", send_buf, actual_data_size);
    }
    updateRobotsStatus();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_multicast");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner spinner(3);

    loadConfig();

    if (openSocket() == -1)
    {
        PERR("openMulticastSocket");
        return -1;
    }

    recv_socket = &multiSocket;

    send_buf[0] = '9'; // Dummy data, indicates that this node doesn't get a valid data from robot

    pub_bs2pc = NH.advertise<comm::mc_in>("bs2pc", 4);

    recv_timer = NH.createTimer(ros::Duration(0.01), recv_cllbck);
    send_timer = NH.createTimer(ros::Duration(0.05), send_cllbck);

    spinner.spin();

    return 0;
}