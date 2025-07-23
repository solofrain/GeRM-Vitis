#include "FreeRTOS.h"

//#include "Register.hpp"
#include "GermaniumRegister.hpp"
#include "PsI2c.hpp"
#include "PsXadc.hpp"
//#include "Network.hpp"
//#include "GermaniumNetwork.hpp"

GermaniumNetwork::GermaniumNetwork( const QueueHandle_t register_single_access_req_queue
                                  , const QueueHandle_t register_single_access_resp_queue
                                  , const QueueHandle_t register_multi_access_req_queue
                                  , const QueueHandle_t register_multi_access_resp_queue
                                  , const QueueHandle_t psi2c0_access_req_queue
                                  , const QueueHandle_t psi2c0_access_resp_queue
                                  , const QueueHandle_t psi2c1_access_req_queue
                                  , const QueueHandle_t psi2c1_access_resp_queue
                                  , const QueueHandle_t psxadc_access_req_queue
                                  , const QueueHandle_t psxadc_access_resp_queue
                                  )
                                  : register_single_access_req_queue_  ( register_single_access_req_queue  )
                                  , register_single_access_resp_queue_ ( register_single_access_resp_queue )
                                  , register_multi_access_req_queue_   ( register_multi_access_req_queue   )
                                  , register_multi_access_resp_queue_  ( register_multi_access_resp_queue  )
                                  , psi2c0_access_req_queue_           ( psi2c0_access_req_queue           )
                                  , psi2c0_access_resp_queue_          ( psi2c0_access_resp_queue          )
                                  , psi2c1_access_req_queue_           ( psi2c1_access_req_queue           )
                                  , psi2c1_access_resp_queue_          ( psi2c1_access_resp_queue          )
                                  , psxadc_access_req_queue_           ( psxadc_access_req_queue           )
                                  , psxadc_access_resp_queue_          ( psxadc_access_resp_queue          )
{}

//===============================================================
// Initialize Rx message process handling map.
//===============================================================
void GermaniumNetwork::rx_instr_map_init()
{
    // Register single access
    this->rx_instr_map_[GermaniumRegister::EVENT_FIFO_CTRL] = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::DETECTOR_TYPE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::MARS_RDOUT_ENB]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::TRIG]            = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::FRAME_NO]        = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::COUNT_TIME_LO]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::COUNT_TIME_HI]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::MARS_CONF_LOAD]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::ADC_SPI]         = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::VERSIONREG]      = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::MARS_PIPE_DELAY] = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::TD_CAL]          = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::COUNT_MODE]      = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::CALPULSE_RATE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::CALPULSE_WIDTH]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::CALPULSE_CNT]    = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::MARS_CALPULSE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::CALPULSE_MODE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::UDP_IP_ADDR]     = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::EVENT_FIFO_CNT]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::EVENT_FIFO_DATA] = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };

    // Local variable update
    this->rx_instr_map_[GermaniumRegister::UPDATE_LOADS]    = [this]( const UdpRxMsg& msg ) { proc_update_loads_msg( msg.payload ); };

    // Register multi access
    this->rx_instr_map_[GermaniumRegister::STUFF_MARS]      = [this]( const UdpRxMsg& msg ) { proc_register_multi_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::AD9252_CNFG]     = [this]( const UdpRxMsg& msg ) { proc_register_multi_access_msg( msg ); };
    this->rx_instr_map_[GermaniumRegister::ZDDM_ARM]        = [this]( const UdpRxMsg& msg ) { proc_register_multi_access_msg( msg ); };

    // Peripheral access
    this->rx_instr_map_[GermaniumRegister::HV]              = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_instr_map_[GermaniumRegister::HV_CUR]          = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_instr_map_[GermaniumRegister::TEMP1]           = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_instr_map_[GermaniumRegister::TEMP2]           = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_instr_map_[GermaniumRegister::TEMP3]           = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };

    // XADC access
    rx_instr_map_[GermaniumRegister::TEMP]          = [this]( const UdpRxMsg& msg ) { proc_xadc_access_msg( msg ); };
}


//===============================================================
// Compose Tx message.
//===============================================================
void GermaniumNetwork::tx_msg_proc( const UdpTxMsg &msg )
{
    QueueSetMemberHandle_t active_queue;

    uint16_t msg_leng;

    active_queue = (QueueHandle_t)xQueueSelectFromSet(resp_queue_set_, portMAX_DELAY);

    if ( active_queue == reg_single_access_resp_queue_ )
    {
        RegisterSingleAccessResp reg_single_access_resp;

        xQueueReceive( reg_single_access_resp_queue_, &reg_single_access_resp, 0);
        msg.op = reg_single_access_resp.op;
        msg.reg_single_access_resp_data = reg_single_access_resp.data;
        
        return;
    }

    if ( active_queue == reg_multi_access_resp_queue_ )
    {
        RegisterMultiAccessResp reg_multi_access_resp;

        xQueueReceive( reg_multi_access_resp_queue_, &reg_multi_access_resp, 0);
        msg.op = reg_multi_access_resp.op;
        msg.reg_multi_access_resp_data = reg_multi_access_resp.data;
        
        return;
    }
            
    if ( active_queue == psi2c_access_resp_queue )
    {
        PsI2cAccessResp psi2c_access_resp;

        xQueueReceive(psi2c_access_resp_queue, &psi2c_access_resp, 0);
        msg.op = psi2c_access_resp.op;
        msg.i2c_access_resp_data = psi2c_access_resp.data;
        
        return;
    }

    if ( active_queue == psxadc_access_resp_queue )
    {
        PsXadcAccessResp psxadc_access_resp;

        xQueueReceive(psxadc_access_resp_queue, &psxadc_access_resp, 0);
        msg.op = psxadc_access_resp.op;
        msg.xadc_access_resp_data = psxadc_access_resp.data;

        return;
    }
}//===============================================================


//===============================================================
// Process register single access message.
//===============================================================
void GermaniumNetwork::proc_register_single_access_msg( const UdpRxMsg& msg )
{
    RegisterSingleAccessReq req;
    req.op = msg.op;
    req.data = *(static_cast<uint32_t*>(msg.payload));

    xQueueSend( register_single_access_request_queue
              , req
              , 0UL
              );
}
//===============================================================

//===============================================================


//===============================================================
// Process register multi access message.
//===============================================================
void GermaniumNetwork::proc_register_multi_access_msg( const UdpRxMsg& msg )
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

    xQueueSend( register_multi_access_request_queue
              , req
              , 0UL
              );
}
//===============================================================

//===============================================================
// Process I2C bus access message.
//===============================================================
void GermaniumNetwork::proc_psi2c_access_msg( const UdpRxMsg& msg )
{
    PsI2cAccessReq req;

    req.op = msg.op;

    switch( req.op )
    {
        case GermaniumRegister::HV:
            break;

        default:
            ;
    }

    xQueueSend( psi2c_access_request_queue
              , req
              , 0UL
              );
}
//===============================================================

//===============================================================
// Process XADC access message.
//===============================================================
void GermaniumNetwork::proc_psi2c_access_msg( const UdpRxMsg& msg )
{
    PsXadcAccessReq req;

    req.op = msg.op;

    switch( req.op )
    {
        case GermaniumRegister::TEMP:
            break;

        default:
            ;
    }

    xQueueSend( psxadc_access_request_queue
              , req
              , 0UL
              );
}
//===============================================================

//===============================================================
// Updata loads.
//===============================================================
void GermaniumNetwork::proc_update_loads_msg( const char* loads )
{
    owner_.update_loads( loads );
}
