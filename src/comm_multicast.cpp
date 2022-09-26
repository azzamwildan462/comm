#include <ros/ros.h>
#include <ros/package.h>
#include "comm/global_var.h"
#include "comm/shmem.h"

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

// Shared memory for Incoming / Outcoming multicast data
semShm_t shm_mc_in;
semShm_t shm_mc_out;
char shm_mc_in_data[64];
char shm_mc_out_data[64];

ros::Timer recv_timer;
ros::Timer send_timer;

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

    /* Disable reception of our own multicast */
    opt = 0; // default 0
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

void signal_handler(int sig)
{
    if (shm_mc_in.createdSegment == 1)
        if (shmRemove(shm_mc_in.shmid, shm_mc_in.segptr) == -1)
            printf("Unable to detach memory\n");
    if (shm_mc_out.createdSegment == 1)
        if (shmRemove(shm_mc_out.shmid, shm_mc_out.segptr) == -1)
            printf("Unable to detach memory\n");
    printf("[comm_multicast] Remove shared memory success\n");
    closeSocket();
    ros::shutdown();
}

void recv_cllbck(const ros::TimerEvent &)
{
    // char recv_buffer[128] = "its";
    // int nrecv = recvfrom(recv_socket->socketID, recv_buffer, 128, 0, &src_addr, &addr_len);

    // if (nrecv > 0)
    // {
    //     uint8_t identifier;
    //     memcpy(&identifier, recv_buffer + 3, 1);
    //     identifier -= '0';

    //     // Memory block = BS -> rbt1 -> rbt2 -> rbt3 -> rbt4 -> rbt5
    //     switch (identifier)
    //     {
    //     case 0:
    //         memcpy(shm_mc_in_data, recv_buffer, 43);
    //         break;
    //     case 1:
    //         memcpy(shm_mc_in_data, recv_buffer + 43, 10);
    //         break;
    //     case 2:
    //         memcpy(shm_mc_in_data, recv_buffer + 53, 10);
    //         break;
    //     case 3:
    //         memcpy(shm_mc_in_data, recv_buffer + 63, 10);
    //         break;
    //     case 4:
    //         memcpy(shm_mc_in_data, recv_buffer + 73, 10);
    //         break;
    //     case 5:
    //         memcpy(shm_mc_in_data, recv_buffer + 83, 10);
    //         break;
    //     }
    // }
}

void send_cllbck(const ros::TimerEvent &)
{
    uint16_t coba;
    uint16_t coba2;
    uint16_t coba3;
    shmRead(shm_mc_in_data, 6, shm_mc_in.shmid, shm_mc_in.segptr, shm_mc_in.sem, 0, 0);
    memcpy(&coba, shm_mc_in_data, 2);
    memcpy(&coba2, shm_mc_in_data + 2, 2);
    memcpy(&coba3, shm_mc_in_data + 4, 2);
    printf("exec2 read: clk %ld | data %d %d %d\n", ros::Time::now().toNSec(), coba, coba2, coba3);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_multicast");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner spinner(2);

    signal(SIGINT, signal_handler);
    loadConfig();

    if (init_shared_mem(&shm_mc_in, SHM_STM2PC_KEY, 64) == -1)
        ros::shutdown();
    if (init_shared_mem(&shm_mc_out, SHM_MC_OUT_KEY, 64) == -1)
        ros::shutdown();

    // if (openSocket() == -1)
    // {
    //     PERR("openMulticastSocket");
    //     return -1;
    // }

    recv_timer = NH.createTimer(ros::Duration(0.1), recv_cllbck);
    send_timer = NH.createTimer(ros::Duration(0.05), send_cllbck);

    spinner.spin();

    return 0;
}