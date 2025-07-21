#pragma once

#include <cstdint>
#include <memory>
#include <atomic>
#include <functional>
#include <any>
#include <map>

extern "C" {

#include "lwipopts.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/udp.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/init.h"

#include "netif/xadapter.h"
#include "errno.h"
#include "xparameters.h"
}

//#include "ZynqDetector.hpp"

template < typename DerivedNetwork,
         , typename Owner
         >
class Network
{
private:
    Owner* owner_;

protected:
    uint32_t udp_port_;

    struct netif netif_;
    int sock_;
    struct sockaddr_in sock_addr_;

    //uint8_t ip_addr_[4];
    //uint8_t netmask_[4];
    //uint8_t gateway_[4];
    //uint8_t dns_[4];
    uint8_t mac_addr_[6];

//    socket_t xUDPSocket;

    //std::atomic<bool> svr_ip_addr_lock_ {false};
    //uint8_t svr_ip_addr_[4];
    std::atomic<uint32_t> srv_ip_addr_;

    int32_t udp_socket_;

    using MessageHandler = std::function<void(std::any&)>;
    std::map<int, MessageHandler> rx_msg_map_;

    void msg_map_init();

    void read_network_config( const std::string& filename );
    static void tcpip_init_done( void *arg );
    bool string_to_addr( const std::string& addr_str, uint8_t* addr );
    
    void udp_rx_task();
    void udp_tx_task();

    void rx_msg_proc( std::any& msg );
    //virtual void tx_msg_proc() = 0;
    

public:

    static constexpr uint32_t UDP_PORT = 0x7000;
    static constexpr uint32_t UDP_MSG_ID = 0xbeef;
    //------------------------------
    // UDP message
    //------------------------------
    static constexpr uint16_t MAX_UDP_MSG_LENG = 4096;
    static constexpr uint16_t MAX_UDP_MSG_DATA_LENG = MAX_UDP_MSG_LENG - 4; // length of message data in bytes
    struct UdpRxMsgStruct
    {
        uint16_t id;
        uint16_t op;
        uint32_t data[MAX_UDP_MSG_DATA_LENG >> 2];
    };
    using UdpRxMsg = UdpRxMsg;

    struct UdpTxMsg
    {
        uint16_t id;
        uint16_t op;
        uint32_t data[MAX_UDP_MSG_DATA_LENG >> 2];
    };
    using UdpTxMsg = UdpTxMsgStruct;

    explicit Network( Owner* owner );
    void network_init();
    void create_network_tasks( TaskHandle_t udp_rx_task_handle,
	                           TaskHandle_t udp_tx_task_handle
						     );
};
 
 #include "Network.tpp"
