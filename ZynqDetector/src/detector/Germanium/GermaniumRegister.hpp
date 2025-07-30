#pragma once

#include "Register.hpp"

class GermaniumRegister : public Register<GermaniumRegister>
{
public:
    static constexpr uint16_t MARS_CONF_LOAD    = 0;
    static constexpr uint16_t LEDS              = 1;
    static constexpr uint16_t MARS_CONFIG       = 2;
    static constexpr uint16_t VERSIONREG        = 3;
    static constexpr uint16_t MARS_CALPULSE     = 4;
    static constexpr uint16_t MARS_PIPE_DELAY   = 5;
    static constexpr uint16_t DETECTOR_TYPE     = 6;
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
    static constexpr uint16_t EVENT_FIFO_CTRL   = 26;
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
    static constexpr uint16_t COUNT_TIME_LO     = 53;
    static constexpr uint16_t COUNT_TIME_HI     = 53;
    static constexpr uint16_t FRAME_NO          = 54;
    static constexpr uint16_t COUNT_MODE        = 55;
//    static constexpr uint16_t UPDATE_LOADS      = 100;
//    static constexpr uint16_t STUFF_MARS        = 101;
//    static constexpr uint16_t AD9252_CNFG       = 102;
//    static constexpr uint16_t ZDDM_ARM          = 103;
    static constexpr uint16_t HV                = 150;  // High voltage
    static constexpr uint16_t HV_CUR            = 151;  // High voltage current
    static constexpr uint16_t TEMP1             = 160;  // Temperature 1
    static constexpr uint16_t TEMP2             = 161;  // Temperature 2
    static constexpr uint16_t TEMP3             = 162;  // Temperature 3
    static constexpr uint16_t ZTEP              = 170;  // CPU temperature
    static constexpr uint16_t DAC_INT_REF       = 180;  // Dac7678 internal reference

    static constexpr uint16_t STUFF_MARS        = 190;
    static constexpr uint16_t ADC_CLK_SKEW      = 191;

    GermaniumRegister( uintptr_t                        base_addr
                     , const QueueHandle_t              single_access_req_queue
                     , const QueueHandle_t              single_access_resp_queue
                     , const QueueHandle_t              multi_access_req_queue
                     , const QueueHandle_t              multi_access_resp_queue
                     , const Logger<GermaniumRegister>& logger
                     );

    Register<GermaniumRegister>* base_;

    void create_register_access_tasks_special();

private:
    const QueueHandle_t multi_access_req_queue_;
    const QueueHandle_t multi_access_resp_queue_;

    void create_register_multi_access_task();
    void register_multi_access_task();

    void create_stuff_mars_task();
    void stuff_mars_task();

};
