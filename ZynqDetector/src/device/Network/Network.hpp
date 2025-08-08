#pragma once

#include <cstdint>
#include <memory>
#include <atomic>
#include <functional>
#include <any>
#include <map>

extern "C" {
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "lwip/inet.h"

//#include "lwipopts.h"
//#include "lwip/ip_addr.h"
//#include "lwip/err.h"
//#include "lwip/udp.h"
//#include "lwip/sys.h"
//#include "lwip/init.h"

#include "netif/xadapter.h"
#include "errno.h"
#include "xparameters.h"
}

//#include "ZynqDetector.hpp"
#include "Logger.hpp"

template < typename DerivedNetwork >
class Network
{

public:

    static constexpr uint32_t UDP_PORT        = 0x7000;
    static constexpr uint32_t UDP_REQ_MSG_ID  = 0xCAFE;
    static constexpr uint32_t UDP_RESP_MSG_ID = 0xBEEF;
    //------------------------------
    // UDP message
    //------------------------------
    static constexpr uint16_t MAX_UDP_MSG_LENG = 4096;
    static constexpr uint16_t MAX_UDP_MSG_DATA_LENG = MAX_UDP_MSG_LENG - 4; // length of message data in bytes

    struct UdpReqMsg
    {   
        uint16_t                           id; 
        uint16_t                           op;
        DerivedNetwork::UdpReqMsgPayload   payload;
    };  
    using UdpRxMsg = UdpReqMsg;

    struct UdpRespMsg
    {
        uint16_t                           id;
        uint16_t                           op;
        DerivedNetwork::UdpRespMsgPayload  payload;
    };
    using UdpTxMsg = UdpRespMsg;


    explicit Network( const Logger& logger  );
    void network_init();
    void create_network_tasks();

private:
    const Logger& logger_;

    struct netif netif_;
    struct sockaddr_in local_addr_;
    int32_t udp_socket_;
    uint8_t mac_addr_[6];
    alignas(64) std::atomic<uint32_t> remote_ip_addr_;

    static constexpr UBaseType_t  RX_TASK_PRIORITY   = 10;
    static constexpr uint32_t     RX_TASK_STACK_SIZE = 1000;
    StaticTask_t                  rx_task_tcb_;
    StackType_t                   rx_task_stack_[RX_TASK_STACK_SIZE];
    TaskConfig                    rx_task_cfg_;

    static constexpr uint32_t     TX_TASK_PRIORITY   = 9;
    static constexpr uint32_t     TX_TASK_STACK_SIZE = 1000;
    StaticTask_t                  tx_task_tcb_;
    StackType_t                   tx_task_stack_[RX_TASK_STACK_SIZE];
    TaskConfig                    tx_task_cfg_;

    TaskHandle_t udp_rx_task_handle_;
    TaskHandle_t udp_tx_task_handle_;
    
    void read_network_config( const std::string& filename );
    static void tcpip_init_done( void *arg );
    bool string_to_addr( const std::string& addr_str, uint8_t* addr );

    //void create_network_tasks();
    void udp_rx_task();
    void udp_tx_task();

    void rx_msg_proc( const UdpRxMsg& msg );
    size_t tx_msg_proc( UdpTxMsg& msg );

};
 
 #include "Network.tpp"
