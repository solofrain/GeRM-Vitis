#pragma once

#include "Network.hpp"

template< typename Owner>
class GermaniumNetwork : public Network<Owner>
{
public:
    //===============================================================
    // Message/operation IDs
    //===============================================================
    const uint16_t MARS_CONF_LOAD    = 0;
    const uint16_t LEDS              = 1;
    const uint16_t MARS_CONFIG       = 2;
    const uint16_t VERSIONREG        = 3;
    const uint16_t MARS_CALPULSE     = 4;
    const uint16_t MARS_PIPE_DELAY   = 5;
    const uint16_t MARS_RDOUT_ENB    = 8;
    const uint16_t EVENT_TIME_CNTR   = 9;
    const uint16_t SIM_EVT_SEL       = 10;
    const uint16_t SIM_EVENT_RATE    = 11;
    const uint16_t ADC_SPI           = 12;
    const uint16_t CALPULSE_CNT      = 16;
    const uint16_t CALPULSE_RATE     = 17;
    const uint16_t CALPULSE_WIDTH    = 18;
    const uint16_t CALPULSE_MODE     = 19;
    const uint16_t TD_CAL            = 20;
    const uint16_t EVENT_FIFO_DATA   = 24;
    const uint16_t EVENT_FIFO_CNT    = 25;
    const uint16_t EVENT_FIFO_CNTRL  = 26;
    const uint16_t DMA_CONTROL       = 32;
    const uint16_t DMA_STATUS        = 33;
    const uint16_t DMA_BASEADDR      = 34;
    const uint16_t DMA_BURSTLEN      = 35;
    const uint16_t DMA_BUFLEN        = 36;
    const uint16_t DMA_CURADDR       = 37;
    const uint16_t DMA_THROTTLE      = 38;
    const uint16_t UDP_IP_ADDR       = 40;
    const uint16_t DMA_IRQ_THROTTLE  = 48;
    const uint16_t DMA_IRQ_ENABLE    = 49;
    const uint16_t DMA_IRQ_COUNT     = 50;
    const uint16_t TRIG              = 52;
    const uint16_t COUNT_TIME        = 53;
    const uint16_t FRAME_NO          = 54;
    const uint16_t COUNT_MODE        = 55;
    const uint16_t UPDATE_LOADS      = 100;
    const uint16_t STUFF_MARS        = 101;
    const uint16_t AD9252_CNFG       = 102;
    const uint16_t ZDDM_ARM          = 103;
    const uint16_t HV                = 150;  // High voltage
    const uint16_t HV_CUR            = 151;  // High voltage current
    const uint16_t TEMP1             = 160;  // Temperature 1
    const uint16_t TEMP2             = 161;  // Temperature 2
    const uint16_t TEMP3             = 162;  // Temperature 3
    const uint16_t ZTEP              = 170;  // CPU temperature
    const uint16_t DAC_INT_REF       = 180;  // DAC7678 internal reference

    //===============================================================
    // Data types
    //===============================================================
    typedef struct
    {
        uint16_t  chip_num;
        uint16_t  addr;
        uint32_t  data;
    } AD9252Cfg;
    //-----------------------------
    typedef struct
    {
        uint16_t  mode;
        uint16_t  val;
    } ZDDMArm;
    //-----------------------------
    typedef union
    {
        uint32_t   Reg_single_acc_req_data;
        uint32_t   Loads[12][14];
        ZDDMArm    Zddm_arm_data;
        AD9252Cfg  Ad9252_cfg_data;
        uint32_t   I2c_acc_req_data;
        uint32_t   Xadc_acc_req_data;
    } UDPRxMsgPayload;
    //-----------------------------
    typedef struct
    {
        uint16_t  id;
        uint16_t  op;
        char      payload[sizeof(UDPRxMsgPayload)];
    } UDPRxMsg;
    //-----------------------------
    typedef union 
    {
        ZDDMArm    zddm_arm_data;
        AD9252Cfg  ad9252_cfg_data;
    } RegisterMultiAccessRequestData;
    //-----------------------------
    typedef struct
    {
        uint16_t  op;
        char      data[sizeof(RegisterMultiAccessRequestData)];
    } RegisterMultiAccessRequest;
    //-----------------------------
    typedef union
    {
        uint32_t  register_single_access_response_data;
        uint32_t  psi2c_access_response_data;
        uint32_t  psxadc_access_response_data;
    } UDPTxMsgPayload;
    //-----------------------------
    typedef struct
    {
        uint16_t  id;
        uint16_t  op;
        char      payload[sizeof(UDPTxMsgPayload)];
    } UDPTxMsg;
    //===============================================================    

protected:
    void rx_msg_map_init();
    void tx_msg_proc( const UDPTxMsg &msg );

    void proc_register_single_access_msg( const UDPRxMsg& msg );
    void proc_register_multi_access_msg( const UDPRxMsg& msg );
    void proc_update_loads_msg( const char* loads );

    //======================================
    // Instruction map
    //======================================
    const std::map<int, std::function<void()>> instruction_map {
        { MARS_CONF_LOAD   , [this]() { this->load_mars_conf(); } },
        { LEDS             , [this]() { this->load_mars_conf(); } },
        { MARS_CONFIG      , [this]() { this->load_mars_conf(); } },
        { VERSIONREG       , [this]() { this->load_mars_conf(); } },
        { MARS_CALPULSE    , [this]() { this->load_mars_conf(); } },
        { MARS_PIPE_DELAY  , [this]() { this->load_mars_conf(); } },
        { MARS_RDOUT_ENB   , [this]() { this->load_mars_conf(); } },
        { EVENT_TIME_CNTR  , [this]() { this->load_mars_conf(); } },
        { SIM_EVT_SEL      , [this]() { this->load_mars_conf(); } },
        { SIM_EVENT_RATE   , [this]() { this->load_mars_conf(); } },
        { ADC_SPI          , [this]() { this->load_mars_conf(); } },
        { CALPULSE_CNT     , [this]() { this->load_mars_conf(); } },
        { CALPULSE_RATE    , [this]() { this->load_mars_conf(); } },
        { CALPULSE_WIDTH   , [this]() { this->load_mars_conf(); } },
        { CALPULSE_MODE    , [this]() { this->load_mars_conf(); } },
        { TD_CAL           , [this]() { this->load_mars_conf(); } },
        { EVENT_FIFO_DATA  , [this]() { this->load_mars_conf(); } },
        { EVENT_FIFO_CNT   , [this]() { this->load_mars_conf(); } },
        { EVENT_FIFO_CNTRL , [this]() { this->load_mars_conf(); } },
        { DMA_CONTROL      , [this]() { this->load_mars_conf(); } },
        { DMA_STATUS       , [this]() { this->load_mars_conf(); } },
        { DMA_BASEADDR     , [this]() { this->load_mars_conf(); } },
        { DMA_BURSTLEN     , [this]() { this->load_mars_conf(); } },
        { DMA_BUFLEN       , [this]() { this->load_mars_conf(); } },
        { DMA_CURADDR      , [this]() { this->load_mars_conf(); } },
        { DMA_THROTTLE     , [this]() { this->load_mars_conf(); } },
        { UDP_IP_ADDR      , [this]() { this->load_mars_conf(); } },
        { DMA_IRQ_THROTTLE , [this]() { this->load_mars_conf(); } },
        { DMA_IRQ_ENABLE   , [this]() { this->load_mars_conf(); } },
        { DMA_IRQ_COUNT    , [this]() { this->load_mars_conf(); } },
        { TRIG             , [this]() { this->load_mars_conf(); } },
        { COUNT_TIME       , [this]() { this->load_mars_conf(); } },
        { FRAME_NO         , [this]() { this->load_mars_conf(); } },
        { COUNT_MODE       , [this]() { this->load_mars_conf(); } },
        { UPDATE_LOADS     , [this]() { this->load_mars_conf(); } },
        { STUFF_MARS       , [this]() { this->load_mars_conf(); } },
        { AD9252_CNFG      , [this]() { this->load_mars_conf(); } },
        { ZDDM_ARM         , [this]() { this->load_mars_conf(); } },
        { HV               , [this]() { this->load_mars_conf(); } },
        { HV_CUR           , [this]() { this->load_mars_conf(); } },
        { TEMP1            , [this]() { this->load_mars_conf(); } },
        { TEMP2            , [this]() { this->load_mars_conf(); } },
        { TEMP3            , [this]() { this->load_mars_conf(); } },
        { ZTEP             , [this]() { this->load_mars_conf(); } },
        { DAC_INT_REF      , [this]() { this->load_mars_conf(); } }
      };

    //friend Germanium

};
