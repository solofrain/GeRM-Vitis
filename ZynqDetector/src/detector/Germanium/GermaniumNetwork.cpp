#include "FreeRTOS.h"

//#include "Register.hpp"
#include "GermaniumRegister.hpp"
#include "PsI2c.hpp"
#include "PsXadc.hpp"
//#include "Network.hpp"
#include "GermaniumNetwork.hpp"

template< typename Owner>
GermaniumNetwork<Owner>::GermaniumNetwork( Owner* owner )
    : owner_ ( owner )
{}

//===============================================================
// Initialize Rx message process handling map.
//===============================================================
template< typename Owner>
void GermaniumNetwork<Owner>::rx_msg_map_init()
{
    // Register single access
    this->rx_msg_map_[GermaniumRegister::EVENT_FIFO_CTRL] = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::DETECTOR_TYPE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::MARS_RDOUT_ENB]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::TRIG]            = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::FRAME_NO]        = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::COUNT_TIME_LO]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::COUNT_TIME_HI]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::MARS_CONF_LOAD]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::ADC_SPI]         = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::VERSIONREG]      = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::MARS_PIPE_DELAY] = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::TD_CAL]          = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::COUNT_MODE]      = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::CALPULSE_RATE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::CALPULSE_WIDTH]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::CALPULSE_CNT]    = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::MARS_CALPULSE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::CALPULSE_MODE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::UDP_IP_ADDR]     = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::EVENT_FIFO_CNT]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::EVENT_FIFO_DATA] = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };

    // Local variable update
    this->rx_msg_map_[GermaniumRegister::UPDATE_LOADS]    = [this]( const UdpRxMsg& msg ) { proc_update_loads_msg( msg.payload ); };

    // Register multi access
    this->rx_msg_map_[GermaniumRegister::STUFF_MARS]      = [this]( const UdpRxMsg& msg ) { proc_register_multi_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::AD9252_CNFG]     = [this]( const UdpRxMsg& msg ) { proc_register_multi_access_msg( msg ); };
    this->rx_msg_map_[GermaniumRegister::ZDDM_ARM]        = [this]( const UdpRxMsg& msg ) { proc_register_multi_access_msg( msg ); };

    // Peripheral access
    this->rx_msg_map_[GermaniumRegister::HV]              = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_msg_map_[GermaniumRegister::HV_CUR]          = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_msg_map_[GermaniumRegister::TEMP1]           = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_msg_map_[GermaniumRegister::TEMP2]           = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_msg_map_[GermaniumRegister::TEMP3]           = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };

    // XADC access
    //rx_msg_map_[GermaniumRegister::TEMP]          = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
}


//===============================================================
// Compose Tx message.
//===============================================================
template< typename Owner>
void GermaniumNetwork<Owner>::tx_msg_proc( const UdpTxMsg &msg )
{
    QueueSetMemberHandle_t active_queue;

    uint16_t msg_leng;

    active_queue = (QueueHandle_t)xQueueSelectFromSet(owner_.resp_queue_set_, portMAX_DELAY);

    if ( active_queue == owner_.reg_single_access_resp_queue_ )
    {
        RegisterSingleAccessResp reg_single_acc_resp;

        xQueueReceive( owner_.reg_single_access_resp_queue_, &reg_single_acc_resp, 0);
        msg.op = reg_single_acc_resp.op;
        msg.reg_single_acc_resp_data = reg_single_acc_resp.data;
        
        return;
    }

    if ( active_queue == owner_.reg_multi_access_resp_queue_ )
    {
        RegisterMultiAccessResp reg_multi_acc_resp;

        xQueueReceive( owner_.reg_multi_access_resp_queue_, &reg_multi_acc_resp, 0);
        msg.op = reg_multi_acc_resp.op;
        msg.reg_multi_acc_resp_data = reg_multi_acc_resp.data;
        
        return;
    }
            
    if ( active_queue == owner_.psi2c_acc_resp_queue )
    {
        PsI2cAccessResp psi2c_acc_resp;

        xQueueReceive(owner_.psi2c_acc_resp_queue, &psi2c_acc_resp, 0);
        msg.op = psi2c_acc_resp.op;
        msg.i2c_acc_resp_data = psi2c_acc_resp.data;
        
        return;
    }

    if ( active_queue == owner_.psxadc_acc_resp_queue )
    {
        PsXadcAccessResp psxadc_acc_resp;

        xQueueReceive(owner_.psxadc_acc_resp_queue, &psxadc_acc_resp, 0);
        msg.op = psxadc_acc_resp.op;
        msg.xadc_acc_resp_data = psxadc_acc_resp.data;

        return;
    }
}//===============================================================


//===============================================================
// Process register single access message.
//===============================================================
template< typename Owner>
void GermaniumNetwork<Owner>::proc_register_single_access_msg( const UdpRxMsg& msg )
{
    RegisterSingleAccessReq req;
    req.op = msg.op;
    req.data = *(static_cast<uint32_t*>(msg.payload));

    xQueueSend( owner_.register_single_access_request_queue
              , req
              , 0UL
              );
}
//===============================================================

//===============================================================


//===============================================================
// Process register multi access message.
//===============================================================
template< typename Owner>
void GermaniumNetwork<Owner>::proc_register_multi_access_msg( const UdpRxMsg& msg )
{
    RegisterMultiAccessRequest req;
    size_t data_leng;

    req.op = msg.op;
    switch( req.op )
    {
        case AD9252_CNFG:
            data_leng  = sizeof( Ad9252Cfg );
            break;

        case ZDDM_ARM:
            data_leng = sizeof( ZddmArm );
            break;

        default:
            ;
    }
    
    memcpy( req.data, msg.payload, data_leng );

    xQueueSend( owner_.register_single_access_request_queue
              , req
              , 0UL
              );
}
//===============================================================


//===============================================================
// Updata loads.
//===============================================================
template< typename Owner>
void GermaniumNetwork<Owner>::proc_update_loads_msg( const char* loads )
{
    owner_.update_loads( loads );
}
