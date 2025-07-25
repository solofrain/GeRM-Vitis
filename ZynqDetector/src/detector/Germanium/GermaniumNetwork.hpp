#pragma once

#include "Network.hpp"

class GermaniumRegister;

class GermaniumNetwork : public Network<GermaniumNetwork, GermaniumRegister>
{
public:
    GermaniumNetwork( const QueueHandle_t              register_single_access_req_queue
                    , const QueueHandle_t              register_single_access_resp_queue
                    , const QueueHandle_t              register_multi_access_req_queue
                    , const QueueHandle_t              register_multi_access_resp_queue
                    , const QueueHandle_t              psi2c0_access_req_queue
                    , const QueueHandle_t              psi2c0_access_resp_queue
                    , const QueueHandle_t              psi2c1_access_req_queue
                    , const QueueHandle_t              psi2c1_access_resp_queue
                    , const QueueHandle_t              psxadc_access_req_queue
                    , const QueueHandle_t              psxadc_access_resp_queue
                    , const Logger<GermaniumRegister>& logger
                    );

    //===============================================================
    // Message/operation IDs
    //===============================================================
    static constexpr uint16_t MARS_CONF_LOAD    = 0;
    static constexpr uint16_t LEDS              = 1;
    static constexpr uint16_t MARS_CONFIG       = 2;
    static constexpr uint16_t VERSIONREG        = 3;
    static constexpr uint16_t MARS_CALPULSE     = 4;
    static constexpr uint16_t MARS_PIPE_DELAY   = 5;
    static constexpr uint16_t MARS_RDOUT_ENB    = 8;
    static constexpr uint16_t EVENT_TIME_CNTR   = 9;
    static constexpr uint16_t SIM_EVT_SEL       = 10;
    static constexpr uint16_t SIM_EVENT_RATE    = 11;
    static constexpr uint16_t ADC_SPI           = 12;
    static constexpr uint16_t CALPULSE_CNT      = 16;
    static constexpr uint16_t CALPULSE_RATE     = 17;
    static constexpr uint16_t CALPULSE_WIDTH    = 18;
    static constexpr uint16_t CALPULSE_MODE     = 19;
    static constexpr uint16_t TD_CAL            = 20;
    static constexpr uint16_t EVENT_FIFO_DATA   = 24;
    static constexpr uint16_t EVENT_FIFO_CNT    = 25;
    static constexpr uint16_t EVENT_FIFO_CNTRL  = 26;
    static constexpr uint16_t DMA_CONTROL       = 32;
    static constexpr uint16_t DMA_STATUS        = 33;
    static constexpr uint16_t DMA_BASEADDR      = 34;
    static constexpr uint16_t DMA_BURSTLEN      = 35;
    static constexpr uint16_t DMA_BUFLEN        = 36;
    static constexpr uint16_t DMA_CURADDR       = 37;
    static constexpr uint16_t DMA_THROTTLE      = 38;
    static constexpr uint16_t UDP_IP_ADDR       = 40;
    static constexpr uint16_t DMA_IRQ_THROTTLE  = 48;
    static constexpr uint16_t DMA_IRQ_ENABLE    = 49;
    static constexpr uint16_t DMA_IRQ_COUNT     = 50;
    static constexpr uint16_t TRIG              = 52;
    static constexpr uint16_t COUNT_TIME        = 53;
    static constexpr uint16_t FRAME_NO          = 54;
    static constexpr uint16_t COUNT_MODE        = 55;
    static constexpr uint16_t UPDATE_LOADS      = 100;
    static constexpr uint16_t STUFF_MARS        = 101;
    static constexpr uint16_t AD9252_CNFG       = 102;
    static constexpr uint16_t ZDDM_ARM          = 103;
    static constexpr uint16_t HV                = 150;  // High voltage
    static constexpr uint16_t HV_CUR            = 151;  // High voltage current
    static constexpr uint16_t TEMP1             = 160;  // Temperature 1
    static constexpr uint16_t TEMP2             = 161;  // Temperature 2
    static constexpr uint16_t TEMP3             = 162;  // Temperature 3
    static constexpr uint16_t ZTEP              = 170;  // CPU temperature
    static constexpr uint16_t DAC_INT_REF       = 180;  // Dac7678 internal reference

    std::map<uint32_t, std::function<void(const UdpRxMsg&)>> rx_instr_map_;
    //===============================================================
    // Data types
    //===============================================================
    struct Ad9252CfgStruct
    {
        uint16_t  chip_num;
        uint16_t  addr;
        uint32_t  data;
    };
    using Ad9252Cfg = Ad9252CfgStruct;
    //-----------------------------
    struct ZddmArmStruct
    {
        uint16_t  mode;
        uint16_t  val;
    };
    using ZddmArm = ZddmArmStruct;
    //-----------------------------
    union UdpRxMsgPayloadStruct
    {
        uint32_t   Reg_single_acc_req_data;
        uint32_t   Loads[12][14];
        ZddmArm    Zddm_arm_data;
        Ad9252Cfg  Ad9252_cfg_data;
        uint32_t   I2c_acc_req_data;
        uint32_t   Xadc_acc_req_data;
    };
    using UdpRxMsgPayload = UdpRxMsgPayloadStruct;
    //-----------------------------
    struct UdpRxMsgStruct
    {
        uint16_t  id;
        uint16_t  op;
        char      payload[sizeof(UdpRxMsgPayload)];
    };
    using UdpRxMsg = UdpRxMsgStruct;
    //-----------------------------
    union RegisterMultiAccessRequestDataStruct
    {
        ZddmArm    zddm_arm_data;
        Ad9252Cfg  ad9252_cfg_data;
    };
    using RegisterMultiAccessRequestData = RegisterMultiAccessRequestDataStruct;
    //-----------------------------
    struct RegisterMultiAccessRequestStruct
    {
        uint16_t  op;
        char      data[sizeof(RegisterMultiAccessRequestData)];
    };
    using RegisterMultiAccessRequest = RegisterMultiAccessRequestStruct;
    //-----------------------------
    union UdpTxMsgPayloadStruct
    {
        uint32_t  register_single_access_response_data;
        uint32_t  psi2c_access_response_data;
        uint32_t  psxadc_access_response_data;
    };
    using UdpTxMsgPayload = UdpTxMsgPayloadStruct;

    //-----------------------------
    struct UdpTxMsgStruct
    {
        uint16_t  id;
        uint16_t  op;
        char      payload[sizeof(UdpTxMsgPayload)];
    };
    using UdpTxMsg = UdpTxMsgStruct;
    //===============================================================    

protected:
    void rx_msg_map_init();
    void tx_msg_proc( const UdpTxMsg &msg );

    void proc_register_single_access_msg( const UdpRxMsg& msg );
    void proc_register_multi_access_msg( const UdpRxMsg& msg );
    void proc_psi2c_access_msg( const UdpRxMsg& msg );
    void proc_psxadc_access_msg( const UdpRxMsg& msg );
    void proc_update_loads_msg( const char* loads );

    ////======================================
    //// Instruction map
    ////======================================
    //const std::map<int, std::function<void(UdpRxMsg&)>> rx_instr_map {
    //    { MARS_CONF_LOAD   , [this](UdpRxMsg& msg) { this->load_mars_conf(msg); } },
    //    { LEDS             , [this]() { this->load_mars_conf(); } },
    //    { MARS_CONFIG      , [this]() { this->load_mars_conf(); } },
    //    { VERSIONREG       , [this]() { this->load_mars_conf(); } },
    //    { MARS_CALPULSE    , [this]() { this->load_mars_conf(); } },
    //    { MARS_PIPE_DELAY  , [this]() { this->load_mars_conf(); } },
    //    { MARS_RDOUT_ENB   , [this]() { this->load_mars_conf(); } },
    //    { EVENT_TIME_CNTR  , [this]() { this->load_mars_conf(); } },
    //    { SIM_EVT_SEL      , [this]() { this->load_mars_conf(); } },
    //    { SIM_EVENT_RATE   , [this]() { this->load_mars_conf(); } },
    //    { ADC_SPI          , [this]() { this->load_mars_conf(); } },
    //    { CALPULSE_CNT     , [this]() { this->load_mars_conf(); } },
    //    { CALPULSE_RATE    , [this]() { this->load_mars_conf(); } },
    //    { CALPULSE_WIDTH   , [this]() { this->load_mars_conf(); } },
    //    { CALPULSE_MODE    , [this]() { this->load_mars_conf(); } },
    //    { TD_CAL           , [this]() { this->load_mars_conf(); } },
    //    { EVENT_FIFO_DATA  , [this]() { this->load_mars_conf(); } },
    //    { EVENT_FIFO_CNT   , [this]() { this->load_mars_conf(); } },
    //    { EVENT_FIFO_CNTRL , [this]() { this->load_mars_conf(); } },
    //    { DMA_CONTROL      , [this]() { this->load_mars_conf(); } },
    //    { DMA_STATUS       , [this]() { this->load_mars_conf(); } },
    //    { DMA_BASEADDR     , [this]() { this->load_mars_conf(); } },
    //    { DMA_BURSTLEN     , [this]() { this->load_mars_conf(); } },
    //    { DMA_BUFLEN       , [this]() { this->load_mars_conf(); } },
    //    { DMA_CURADDR      , [this]() { this->load_mars_conf(); } },
    //    { DMA_THROTTLE     , [this]() { this->load_mars_conf(); } },
    //    { UDP_IP_ADDR      , [this]() { this->load_mars_conf(); } },
    //    { DMA_IRQ_THROTTLE , [this]() { this->load_mars_conf(); } },
    //    { DMA_IRQ_ENABLE   , [this]() { this->load_mars_conf(); } },
    //    { DMA_IRQ_COUNT    , [this]() { this->load_mars_conf(); } },
    //    { TRIG             , [this]() { this->load_mars_conf(); } },
    //    { COUNT_TIME       , [this]() { this->load_mars_conf(); } },
    //    { FRAME_NO         , [this]() { this->load_mars_conf(); } },
    //    { COUNT_MODE       , [this]() { this->load_mars_conf(); } },
    //    { UPDATE_LOADS     , [this]() { this->load_mars_conf(); } },
    //    { STUFF_MARS       , [this]() { this->load_mars_conf(); } },
    //    { AD9252_CNFG      , [this]() { this->load_mars_conf(); } },
    //    { ZDDM_ARM         , [this]() { this->load_mars_conf(); } },
    //    { HV               , [this]() { this->load_mars_conf(); } },
    //    { HV_CUR           , [this]() { this->load_mars_conf(); } },
    //    { TEMP1            , [this]() { this->load_mars_conf(); } },
    //    { TEMP2            , [this]() { this->load_mars_conf(); } },
    //    { TEMP3            , [this]() { this->load_mars_conf(); } },
    //    { ZTEP             , [this]() { this->load_mars_conf(); } },
    //    { DAC_INT_REF      , [this]() { this->load_mars_conf(); } }
    //  };

    //friend Germanium
private:

    Network<GermaniumNetwork, GermaniumRegister>* base_;

    const Logger<GermaniumRegister>& logger_;

    QueueHandle_t& register_single_access_req_queue_;
    QueueHandle_t& register_single_access_resp_queue_;
    QueueHandle_t& register_multi_access_req_queue_;
    QueueHandle_t& register_multi_access_resp_queue_;
    QueueHandle_t& psi2c0_access_req_queue_;
    QueueHandle_t& psi2c0_access_resp_queue_;
    QueueHandle_t& psi2c1_access_req_queue_;
    QueueHandle_t& psi2c1_access_resp_queue_;
    QueueHandle_t& psxadc_access_req_queue_;
    QueueHandle_t& psxadc_access_resp_queue_;
};


