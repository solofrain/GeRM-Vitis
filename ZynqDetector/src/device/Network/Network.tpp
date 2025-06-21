#pragma once

extern "C" {
#include "lwip/tcpip.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
}

#include "Logger.hpp"

template <typename Owner>
Network<Owner>::Network( Owner* owner, uint32_t udp_port )
	                   : owner_    ( owner )
                       , udp_port_ ( udp_port )
{}

//===============================================================
// Network initialization.
//===============================================================

template <typename Owner>
void Network<Owner>::tcpip_init_done(void *arg)
{
    if (arg)
    {
        *((char *)arg) = 1;
    }
}


template <typename Owner>
void Network<Owner>::network_init()
{
    int setup = 0;

        // Initialize lwIP stack (TCP/IP thread + netif)

    lwip_init();


    /* Add network interface to the netif_list, and set it as default */
    if (!xemac_add( &netif_,
                    NULL,
                    NULL,
                    NULL,
                    mac_addr_,
                    PLATFORM_EMAC_BASEADDR) )
    {
        xil_printf("Error adding network interface\r\n");
        return;
    }

    netif_set_default( &netif_ );

    /* specify that the network if is up */
    netif_set_up( &netif_ );

    read_network_config( "config" );


    if ( ( sock_ = socket( AF_INET, SOCK_DGRM, 0 ) ) < 0 )
    {
        log_err( "Failed to create socket." );
        return;
    }

    memset( &sock_addr_, 0, sizeof(struct sockaddr_in) );
    sock_addr_.sin_family      = AF_INET;
    sock_addr_.sin_port        = htons(UDP_CONN_PORT);
    sock_addr_.sin_addr.s_addr = htonl(INADDR_ANY);

    if ( bind( sock_, (struct sockaddr *)&sock_addr_, sizeof(sock_addr_) != ERR_OK) )
    {
        log_error( "Error on bind" );
        close( sock_ );
        return;
    }

}
//==============================================================


//===============================================================
// Read network parameters from file:
// - IP address
// - Netmask
// - Gateway
// - DNS server
// - MAC address
//===============================================================
template <typename Owner>
void Network<Owner>::read_network_config( const std::string& filename )
{
    FATFS fs;    // File system object
    FRESULT res; // Result code
    FIL file;
    UINT br;

    char buff[50];
    int buff_index = 0;
    
    res = f_mount(&fs, "", 1); // Mount the default drive
    if (res != FR_OK)
    {
        throw std::runtime_error( "Failed to mount SD card" );
    }

    res = f_open(&file, "filename.txt", FA_READ | FA_WRITE);
    if (res != FR_OK)
    {
        throw std::runtime_error( "Failed to open config file" );
    }


    std::ifstream file( filename );
    if ( !file.is_open() )
    {
        throw std::runtime_error( "Failed to open config file" );
        std::cerr << "Error: could not open " << filename << "!\n";
        return;
    }

    std::string line;

    //while( std::getline(file, line) )
    while (f_read(&file, &buff[buff_index], 1, &br) == FR_OK && br > 0)
    {
        if ( buff[buff_index] != '\n' && buff[buff_index] != '\r')
        {
            ++buff_index;
            continue;
        }
        
        buff[buff_index] = '\0'; // Null-terminate the line

        std::istringstream stream( buff );
        std::string key, value;

        stream >> key >> value;

        //if ( key == "ip-address" && !string_to_addr(value, ip_addr.begin() ) )
        //{
        //    throw NetException( "Invalid IP address format", value );
        //}
        //else if ( key == "netmask" && !string_to_addr( value, netmask.begin() ) )
        //{
        //    throw NetException( "Invalid netmask format", value );
        //}
        //else if ( key == "gateway" && !string_to_addr( value, gateway.begin() ) )
        //{
        //    throw NetException( "Invalid gateway format", value );
        //}
        //else if ( key == "dns" && !string_to_addr( value, dns.begin() ))
        //{
        //    throw NetException( "Invalid DNS format", value );
        //}
        //else if ( key == "mac-address" && !string_to_addr( value, mac_addr.begin() ) )
        //{
        //    throw NetException( "Invalid MAC address format", value );
        //}

        if ( key == "ip-address" )
            if ( !inet_aton(value, ip_addr_ ) )
            {
                throw NetException( "Invalid IP address format", value );
            }
        else if ( key == "netmask" )
            if ( !inet_aton( value, netmask_ ) )
            {
                throw NetException( "Invalid netmask format", value );
            }
        else if ( key == "gateway" )
            if ( !inet_aton( value, gateway_ ) )
            {
                throw NetException( "Invalid gateway format", value );
            }
        else if ( key == "dns" )
            if ( !inet_aton( value, dns_ ) )
            {
                throw NetException( "Invalid DNS format", value );
            }
        else if ( key == "mac-address" )
            if ( !string_to_addr( value, mac_addr.begin() ) )
            {
                throw NetException( "Invalid MAC address format", value );
            }

        memset( buff, sizeof( buff ), 0 );
        buff_index = 0;
    }

    f_close( &file );
    f_mount(NULL, "", 1);
}
//===============================================================


//===============================================================
// Convert a string to an IP/MAC address.
//===============================================================
template <typename Owner>
bool Network<Owner>::string_to_addr(
    const std::string& addr_str,
    uint8_t* addr
)
{
    std::stringstream ss( addr_str );

    bool is_ip = ( addr_str.find( '.' ) != std::string::npos );
    auto separator = is_ip ? '.' : ':';
    auto num_separator = std::count( addr_str.begin(), addr_str.end(), separator );
    if ( ( is_ip && num_separator != 3 ) || ( !is_ip && num_separator != 5 ) )
    {
        std::cerr << "Wrong address string format" << addr_str << '\n';
        return false;
    }

    std::string segment;

    int i = 0;

    while ( std::getline ( ss, segment, separator ) )
    {
        int byte = 0;
        if ( is_ip )
        {
            if ( std::stringstream( segment ) >> byte )
            {
                if (byte < 0 || byte > 255) return false;  // Invalid byte value
            }
            else
            {
                return false;  // Invalid segment
            }
        }
        else
        {
            try
            {
                byte = std::stoi( segment, nullptr, 16 );
                if (byte < 0 || byte > 255) return false;  
            }
            catch ( const std::invalid_argument& )
            {
                return false;  // Invalid hex segment
            }            
        }
        *(addr++) = static_cast<uint8_t>( byte );
    }

    return true;
}
//===============================================================


//===============================================================
// Create Tx and Rx tasks
//===============================================================
template <typename Owner>
void Network<Owner>::create_network_tasks
(
    TaskHandle_t udp_rx_task_handle,
    TaskHandle_t udp_tx_task_handle
)
{
    auto task_func = std::make_unique<std::function<void()>>([this]() { udp_rx_task(); });
    xTaskCreate( task_wrapper, "UDP Rx", 1000, &task_func, 1, udp_rx_task_handle );

    auto task_func = std::make_unique<std::function<void()>>([this]() { udp_tx_task(); });
    xTaskCreate( task_wrapper, "UDP Tx", 1000, &task_func, 1, udp_tx_task_handle );
}
//===============================================================

//===============================================================
// UDP receive task.
//===============================================================
template <typename Owner>
void Network<Owner>::udp_rx_task()
{
    UDPRxMsg msg;
    uint32_t msg_leng;
    
    struct freertos_sockaddr src_sock_addr;
    socklen_t src_addr_leng = sizeof( src_sock_addr );

    uint32_t remote_ip_addr, remote_ip_addr_tmp;

    while(1)
    {
        uint16_t msg_leng = FreeRTOS_recvfrom( udp_socket_
                                             , &msg
                                             , sizeof( msg )
                                             , 0
                                             , ( struct freertos_sockaddr * ) &src_sock_addr
                                             , &src_addr_leng );

        if ( (msg_leng <= 0) || (msg.id != UDP_MSG_ID) )
        {
            // error report
            continue;
        }

        remote_ip_addr_tmp = FreeRTOS_ntohl(src_addr.sin_addr);
        if ( remote_ip_addr_tmp != remote_ip_addr )
        {
            // update server IP address
            remote_ip_addr = remote_ip_addr_tmp;
            memcpy( svr_ip_addr_, &remote_ip_addr, 4 );
        }
        
        rx_msg_proc( msg );
    }
}
//===============================================================


//===============================================================
// UDP transmit task.
//===============================================================
template <typename Owner>
void Network<Owner>::udp_tx_task()
{
    uint16_t msg_leng, tx_length;
    UDPTxMsg msg;

    while(1)
    {
        msg_leng = tx_msg_proc(msg);
        tx_length = FreeRTOS_sendto( udp_socket_
                                   , msg
                                   , msg_leng
                                   , 0
                                   , &dest_sock_addr
                                   , sizeof(dest_sock_addr) );

        if (tx_length < 0)
        {
            log_error( "Failed to send UDP message\n" );
        }
    }
}
//===============================================================


//===============================================================
// UDP Rx message processing.
//===============================================================
template <typename Owner>
void Network<Owner>::rx_msg_proc( std::any& msg )
{
    //int instr = msg.op && 0x7FFF;
    auto it = rx_msg_map_.find(msg.op && 0x7FFF);
    if (it != rx_msg_map_.end())
    {
        it->second(msg);  // Call the corresponding function
    }
    else
    {
        std::cout << "Unknown instruction: " << instr << '\n';
    }
}
//===============================================================
