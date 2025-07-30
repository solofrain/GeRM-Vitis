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
                    //, const QueueHandle_t              psi2c0_access_resp_queue
                    , const QueueHandle_t              psi2c1_access_req_queue
                    , const QueueHandle_t              psi2c_access_resp_queue
                    , const QueueHandle_t              psxadc_access_req_queue
                    , const QueueHandle_t              psxadc_access_resp_queue
                    , const QueueHandle_t              ad9252_access_req_queue
                    , const QueueHandle_t              mars_access_req_queue
                    , const QueueHandle_t              zddm_access_req_queue
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
    //static constexpr uint16_t STUFF_MARS        = 101;
    //static constexpr uint16_t AD9252_CNFG       = 102;
    //static constexpr uint16_t ZDDM_ARM          = 103;
    static constexpr uint16_t HV                = 150;  // High voltage
    static constexpr uint16_t HV_CUR            = 151;  // High voltage current
    static constexpr uint16_t TEMP1             = 160;  // Temperature 1
    static constexpr uint16_t TEMP2             = 161;  // Temperature 2
    static constexpr uint16_t TEMP3             = 162;  // Temperature 3
    static constexpr uint16_t ZTEP              = 170;  // CPU temperature
    static constexpr uint16_t DAC_INT_REF       = 180;  // Dac7678 internal reference

    static constexpr uint16_t STUFF_MARS        = 190;
    static constexpr uint16_t ADC_CLK_SKEW      = 191;
    static constexpr uint16_t ZDDM_ARM          = 192;


    Network<GermaniumNetwork, GermaniumRegister>* base_;

    std::map<uint32_t, std::function<void(const UdpRxMsg&)>> rx_instr_map_;
    //===============================================================
    // Data types
    //===============================================================
    struct __attribute__((__packed__)) Ad9252CfgStruct
    {
        uint16_t  chip_num;
        //uint16_t  addr;
        uint16_t  data;
    };
    using Ad9252Cfg = Ad9252CfgStruct;
    using Ad9252AccessReq = Ad9252CfgStruct;
    //-----------------------------
    struct __attribute__((__packed__)) ZddmArmStruct
    {
        uint16_t  mode;
        uint16_t  val;
    };
    using ZddmArm = ZddmArmStruct;
    using ZddmAccessReq = ZddmArmStruct;
    //-----------------------------
    struct MarsLoadStruct
    {
        uint32_t  loads[12][14];
    };
    using MarsLoad = MarsLoadStruct;
    using MarsAccessReq = MarsLoadStruct;
    //-----------------------------
    union UdpRxMsgPayloadStruct
    {
        uint32_t   reg_single_acc_req_data;
        ZddmArm    zddm_arm_data;
        Ad9252Cfg  ad9252_cfg_data;
        MarsLoad   mars_load_data;
        uint32_t   i2c_acc_req_data;
        uint32_t   xadc_acc_req_data;
    };
    using UdpRxMsgPayload = UdpRxMsgPayloadStruct;
    //-----------------------------
    //struct UdpRxMsgStruct
    //{
    //    uint16_t  id;
    //    uint16_t  op;
    //    char      payload[sizeof(UdpRxMsgPayload)];
    //};
    //using UdpRxMsg = UdpRxMsgStruct;
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
        uint16_t                       op;
        RegisterMultiAccessRequestData data;
    };
    using RegisterMultiAccessRequest = RegisterMultiAccessRequestStruct;
    //-----------------------------
    union UdpTxMsgPayloadStruct
    {
        uint32_t  register_single_access_response_data;
        uint32_t  register_multi_access_response_data;
        uint32_t  psi2c_access_response_data;
        uint32_t  psxadc_access_response_data;
    };
    using UdpTxMsgPayload = UdpTxMsgPayloadStruct;

    //-----------------------------
    //struct UdpTxMsgStruct
    //{
    //    uint16_t  id;
    //    uint16_t  op;
    //    char      payload[sizeof(UdpTxMsgPayload)];
    //};
    //using UdpTxMsg = UdpTxMsgStruct;
    //===============================================================    

    size_t tx_msg_proc( UdpTxMsg &msg );

protected:
    void rx_msg_map_init();

    void proc_register_single_access_msg( const UdpRxMsg& msg );
    //void proc_register_multi_access_msg ( const UdpRxMsg& msg );
    void proc_psi2c_access_msg          ( const UdpRxMsg& msg );
    void proc_psxadc_access_msg         ( const UdpRxMsg& msg );
    //void proc_update_loads_msg          ( const char* loads );
    void proc_ad9252_access_msg         ( const UdpRxMsg& msg );
    void proc_mars_access_msg           ( const UdpRxMsg& msg );
    void proc_zddm_access_msg           ( const UdpRxMsg& msg );

private:


    const Logger<GermaniumRegister>& logger_;

    QueueHandle_t& register_single_access_req_queue_;
    QueueHandle_t& register_single_access_resp_queue_;
    QueueHandle_t& register_multi_access_req_queue_;
    QueueHandle_t& register_multi_access_resp_queue_;
    QueueHandle_t& psi2c0_access_req_queue_;
    //QueueHandle_t& psi2c0_access_resp_queue_;
    QueueHandle_t& psi2c1_access_req_queue_;
    QueueHandle_t& psi2c_access_resp_queue_;
    QueueHandle_t& psxadc_access_req_queue_;
    QueueHandle_t& psxadc_access_resp_queue_;
    QueueHandle_t& ad9252_access_req_queue_;
    QueueHandle_t& mars_access_req_queue_;
    QueueHandle_t& zddm_access_req_queue_;
};


