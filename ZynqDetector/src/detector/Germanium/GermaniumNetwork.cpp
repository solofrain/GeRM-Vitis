#include "FreeRTOS.h"

#include "Register.hpp"
#include "PsI2c.hpp"
#include "PsXadc.hpp"
//#include "Network.hpp"
//#include "GermaniumNetwork.hpp"

GermaniumNetwork::GermaniumNetwork( const QueueHandle_t   register_single_access_req_queue
                                  , const QueueHandle_t   register_single_access_resp_queue
                                  , const QueueHandle_t   psi2c0_access_req_queue
                                  , const QueueHandle_t   psi2c1_access_req_queue
                                  , const QueueHandle_t   psi2c_access_resp_queue
                                  , const QueueHandle_t   psxadc_access_req_queue
                                  , const QueueHandle_t   psxadc_access_resp_queue
                                  , const QueueHandle_t   ad9252_access_req_queue
                                  , const QueueHandle_t   mars_access_req_queue
                                  , const QueueHandle_t   zddm_access_req_queue
                                  , const Logger&         logger
                                  )
                                  : Network<GermaniumNetwork>           ( logger                            )
                                  , register_single_access_req_queue_   ( register_single_access_req_queue  )
                                  , register_single_access_resp_queue_  ( register_single_access_resp_queue )
                                  , psi2c0_access_req_queue_            ( psi2c0_access_req_queue           )
                                  , psi2c1_access_req_queue_            ( psi2c1_access_req_queue           )
                                  , psi2c_access_resp_queue_            ( psi2c_access_resp_queue           )
                                  , psxadc_access_req_queue_            ( psxadc_access_req_queue           )
                                  , psxadc_access_resp_queue_           ( psxadc_access_resp_queue          )
                                  , ad9252_access_resp_queue_           ( ad9252_access_resp_queue          )
                                  , mars_access_resp_queue_             ( mars_access_resp_queue            )
                                  , zddm_access_resp_queue_             ( zddm_access_resp_queue            )
                                  , logger_                             ( logger                            )
                                  , base_           ( static_cast<Network<GermaniumNetwork>*>(this)
{
    rx_instr_map_init();
}


//===============================================================
// Initialize Rx message process handling map.
//===============================================================
void GermaniumNetwork::rx_instr_map_init()
{
    // Register single access
    this->rx_instr_map_[EVENT_FIFO_CTRL] = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[DETECTOR_TYPE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[MARS_RDOUT_ENB]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[TRIG]            = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[FRAME_NO]        = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[COUNT_TIME_LO]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[COUNT_TIME_HI]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[MARS_CONF_LOAD]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[ADC_SPI]         = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[VERSIONREG]      = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[MARS_PIPE_DELAY] = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[TD_CAL]          = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[COUNT_MODE]      = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[CALPULSE_RATE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[CALPULSE_WIDTH]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[CALPULSE_CNT]    = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[MARS_CALPULSE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[CALPULSE_MODE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[UDP_IP_ADDR]     = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[EVENT_FIFO_CNT]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[EVENT_FIFO_DATA] = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };

    // Local variable update
    this->rx_instr_map_[UPDATE_LOADS]    = [this]( const UdpRxMsg& msg ) { proc_update_loads_msg( msg.payload ); };

    // Register multi access
    this->rx_instr_map_[STUFF_MARS]      = [this]( const UdpRxMsg& msg ) { proc_mars_access_msg( msg ); };
    this->rx_instr_map_[AD9252_CNFG]     = [this]( const UdpRxMsg& msg ) { proc_ad9252_access_msg( msg ); };
    this->rx_instr_map_[ZDDM_ARM]        = [this]( const UdpRxMsg& msg ) { proc_zddm_access_msg( msg ); };

    // Peripheral access
    this->rx_instr_map_[HV]              = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_instr_map_[HV_CUR]          = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_instr_map_[TEMP1]           = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_instr_map_[TEMP2]           = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_instr_map_[TEMP3]           = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };

    // XADC access
    rx_instr_map_[TEMP]          = [this]( const UdpRxMsg& msg ) { proc_xadc_access_msg( msg ); };
}




//===============================================================
// Process register single access message.
//===============================================================
void GermaniumNetwork::proc_register_single_access_msg( const UdpRxMsg& msg )
{
    RegisterSingleAccessReq req;
    req.op = msg.op;
    req.data = *(static_cast<uint32_t*>(msg.payload));

    xQueueSend( register_single_access_req_queue_
              , req
              , 0UL
              );
}
//===============================================================



//===============================================================
// Process AD9252 access message.
//===============================================================
void GermaniumNetwork::proc_ad9252_access_msg( const UdpRxMsg& msg )
{
    Ad9252AccessReq req;
    req.chip_num = msg.payload.ad9252_cfg_data.chip_num;
    req.data     = msg.payload.ad9252_cfg_data.data;

    xQueueSend( ad9252_access_req_queue__
              , req
              , 0UL
              );
}
//===============================================================

//===============================================================
// Process MARS access message.
//===============================================================
void GermaniumNetwork::proc_mars_access_msg( const UdpRxMsg& msg )
{
    MarsAccessReq req;
    req.loads = msg.payload.mars_load_data.loads;

    xQueueSend( mars_access_req_queue__
              , req
              , 0UL
              );
}
//===============================================================

//===============================================================
// Process ZDDM access message.
//===============================================================
void GermaniumNetwork::proc_zddm_access_msg( const UdpRxMsg& msg )
{
    ZddmAccessReq req;
    req.mode = msg.payload.zddm_arm_data.mode;
    req.val  = msg.payload.zddm_arm_data.val;

    xQueueSend( zddm_access_req_queue__
              , req
              , 0UL
              );
}
//===============================================================



////===============================================================
//// Process register multi access message.
////===============================================================
//void GermaniumNetwork::proc_register_multi_access_msg( const UdpRxMsg& msg )
//{
//    RegisterMultiAccessRequest req;
//    size_t data_leng;
//
//    req.op = msg.op;
//    switch( req.op )
//    {
//        case AD9252_CNFG:
//            data_leng  = sizeof( Ad9252Cfg );
//            break;
//
//        case ZDDM_ARM:
//            data_leng = sizeof( ZddmArm );
//            break;
//
//        default:
//            ;
//    }
//    
//    memcpy( req.data, msg.payload, data_leng );
//
//    xQueueSend( register_multi_access_req_queue_
//              , req
//              , 0UL
//              );
//}
////===============================================================

//===============================================================
// Process I2C bus access message.
//===============================================================
void GermaniumNetwork::proc_psi2c_access_msg( const UdpRxMsg& msg )
{
    PsI2cAccessReq req;

    req.op = msg.op;

    switch( req.op )
    {
        case HV:
            break;

        default:
            ;
    }

    xQueueSend( psi2c_access_req_queue_
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
        case TEMP:
            break;

        default:
            ;
    }

    xQueueSend( psxadc_access_req_queue_
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

//===============================================================
// Compose Tx message.
//===============================================================
size_t GermaniumNetwork::tx_msg_proc( UdpTxMsg& msg )
{
    QueueSetHandle_t resp_queue_set;
    resp_queue_set = xQueueCreateSet(50);

    xQueueAddToSet( register_single_access_resp_queue_, resp_queue_set );
    xQueueAddToSet( psi2c_resp_queue_, resp_queue_set );
    xQueueAddToSet( psxadc_resp_queue_, resp_queue_set );

    QueueSetMemberHandle_t active_queue;

    //uint16_t msg_leng;

    active_queue = (QueueHandle_t)xQueueSelectFromSet(resp_queue_set, portMAX_DELAY);

    if ( active_queue == reg_single_access_resp_queue_ )
    {
        RegisterSingleAccessResp reg_single_access_resp;

        xQueueReceive( reg_single_access_resp_queue_, &reg_single_access_resp, 0);
        msg.op = reg_single_access_resp.op;
        msg.reg_single_access_resp_data = reg_single_access_resp.data;
        
        return (4+sizeof(msg.reg_single_access_resp_data);
    }

    if ( active_queue == psi2c_access_resp_queue )
    {
        PsI2cAccessResp psi2c_access_resp;

        xQueueReceive(psi2c_access_resp_queue_, &psi2c_access_resp, 0);
        msg.op = psi2c_access_resp.op;
        msg.i2c_access_resp_data = psi2c_access_resp.data;
        
        return (4+sizeof(msg.i2c_access_resp_data);
    }

    if ( active_queue == psxadc_access_resp_queue )
    {
        PsXadcAccessResp psxadc_access_resp;

        xQueueReceive(psxadc_access_resp_queue_, &psxadc_access_resp, 0);
        msg.op = psxadc_access_resp.op;
        msg.xadc_access_resp_data = psxadc_access_resp.data;

        return (4+sizeof(msg.xdc_access_resp_data);
    }
}//===============================================================
