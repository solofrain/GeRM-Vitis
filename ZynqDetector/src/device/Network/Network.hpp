#pragma once

#include <cstdint>
#include <memory>
#include <atomic>
#include <functional>
#include <any>
#include <map>

#include "ZynqDetector.hpp"

class Network
{
protected:
    uint32_t udp_port_;

    uint8_t ip_addr_[4];
    uint8_t netmask_[4];
    uint8_t gateway_[4];
    uint8_t dns_[4];
    uint8_t mac_addr_[6];

//    socket_t xUDPSocket;

    std::atomic<bool> svr_ip_addr_lock_ {false};
    uint8_t svr_ip_addr_[4];

    int32_t udp_socket_;

    using MessageHandler = std::function<void(std::any&)>;
    std::map<int, MessageHandler> rx_msg_map_;

    virtual void msg_map_init() = 0;

    void read_network_config( const std::string& filename );
    bool string_to_addr( const std::string& addr_str, uint8_t* addr );
    
    virtual void udp_rx_task();
    virtual void udp_tx_task();

    virtual void rx_msg_proc();
    virtual void tx_msg_proc() = 0;
    

public:

    //------------------------------
    // UDP message
    //------------------------------
    static constexpr uint16_t MAX_UDP_MSG_LENG = 4096;
    static constexpr uint16_t MAX_UDP_MSG_DATA_LENG = MAX_UDP_MSG_LENG - 4; // length of message data in bytes
    typedef struct
    {
        uint16_t id;
        uint16_t op;
        uint32_t data[MAX_UDP_MSG_DATA_LENG >> 2];
    } UDPRxMsg;

    typedef struct
    {
        uint16_t id;
        uint16_t op;
        uint32_t data[MAX_UDP_MSG_DATA_LENG >> 2];
    } UDPTxMsg;

    explicit Network( int udp_port );
    void network_init();
    void create_network_tasks( TaskHandle_t udp_rx_task_handle,
	                           TaskHandle_t udp_tx_task_handle
						     );
};
 
