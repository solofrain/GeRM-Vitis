
#include <cstdint>
#include <string>
#include <map>
#include <utility>

#include "ZynqDetector.hpp"
#include "GermaniumNetwork.hpp"
#include "GermaniumZynq.hpp"
#include "GermaniumRegister.hpp"

#include "Ltc2309.hpp"
#include "PsI2c.hpp"
#include "Dac7678.hpp"
#include "Tmp100.hpp"



// const uint16_t LKUPADDRREG       = ?;

using Instruction_Handler = std::function<void()>;

struct germ_udp_msg_t
{
    uint32_t data;
};

class GermaniumDetector : public ZynqDetector< GermaniumDetector
                                             , GermaniumNetwork<GermaniumDetector>
                                             , GermaniumZynq
                                             , GermaniumRegister
                                             >
{

protected:
    // void greate_tasks();
    TaskHandle_t psi2c_0_task_handler_;
    TaskHandle_t psi2c_1_task_handler_;
    TaskHandle_t psxadc_task_handler_;

    QueueHandle_t psi2c_0_req_queue;
    QueueHandle_t psi2c_1_req_queue;
    QueueHandle_t psxadc_req_queue;
    QueueHandle_t register_multi_access_req_queue;

    QueueHandle_t psi2c_0_resp_queue;
    QueueHandle_t psi2c_1_resp_queue;
    QueueHandle_t psxadc_resp_queue;
    QueueHandle_t register_multi_access_resp_queue;


    std::unique_ptr<Ltc2309<PsI2c>> ltc2309_0_, ltc2309_1_;
    std::unique_ptr<Dac7678<PsI2c>> dac7678_;
    std::unique_ptr<Tmp100<PsI2c>>  tmp100_0_;
    std::unique_ptr<Tmp100<PsI2c>>  tmp100_1_;
    std::unique_ptr<Tmp100<PsI2c>>  tmp100_2_;

    int num_chips_;
    int nelm_;

    // ASIC configuration data
    uint16_t loads[12][14];
  
    const uint8_t LTC2309_0_I2C_ADDR = 0x08;
    const uint8_t LTC2309_1_I2C_ADDR = 0x0A;
    const uint8_t DAC7678_I2C_ADDR = 0x1A;
    const uint8_t TMP100_0_I2C_ADDR = 0x48;
    const uint8_t TMP100_1_I2C_ADDR = 0x49;
    const uint8_t TMP100_2_I2C_ADDR = 0x59;

    const uint16_t VL0 = 0;
    const uint16_t VL1 = 1;
    const uint16_t VH1 = 2;
    const uint16_t VL2 = 3;
    const uint16_t VH2 = 4;
    const uint16_t HV  = 5;
    const uint16_t P1  = 6;
    const uint16_t P2  = 7;

    const uint16_t TEMP1   = 0;
    const uint16_t TEMP2   = 1;
    const uint16_t TEMP3   = 2;
    const uint16_t TEMP4   = 3;
    const uint16_t HV_RBV  = 4;
    const uint16_t HV_CURR = 5;
    //const uint16_t P1      = 
    //const uint16_t P2      = 
    const uint16_t ILEAK   = 8;
    const uint16_t P_V     = 9;

    //const uint16_t TEMP1 = 
    //const uint16_t TEMP2 = 
    //const uint16_t TEMP3 = 

    // DAC process map: <op, <device, channel>>
    const std::map<uint16_t, std::pair<std::shared_ptr<Dac7678<PsI2c>>, uint8_t>> dac_instr_map = 
        {
          { VL0, std::make_pair( dac7678_, 0 ) },
          { VL1, std::make_pair( dac7678_, 1 ) },
          { VH1, std::make_pair( dac7678_, 2 ) },
          { VL2, std::make_pair( dac7678_, 3 ) },
          { VH2, std::make_pair( dac7678_, 4 ) },
          { HV,  std::make_pair( dac7678_, 5 ) },
          { P1,  std::make_pair( dac7678_, 6 ) },
          { P2,  std::make_pair( dac7678_, 7 ) }
        };

    // ADC process map: <op, <device, channel>>
    const std::map<uint16_t, std::pair<std::shared_ptr<Ltc2309<PsI2c>>, uint8_t>> adc_instr_map = 
        {
          { TEMP1,   std::make_pair( ltc2309_0_, 0 ) },
          { TEMP2,   std::make_pair( ltc2309_0_, 1 ) },
          { TEMP3,   std::make_pair( ltc2309_0_, 2 ) },
          { TEMP4,   std::make_pair( ltc2309_0_, 3 ) },
          { HV_RBV,  std::make_pair( ltc2309_0_, 4 ) },
          { HV_CURR, std::make_pair( ltc2309_0_, 5 ) },
          { P1,      std::make_pair( ltc2309_0_, 6 ) },
          { P2,      std::make_pair( ltc2309_0_, 7 ) },
          { ILEAK,   std::make_pair( ltc2309_1_, 0 ) },
          { P_V,     std::make_pair( ltc2309_1_, 1 ) }
        };

    // Temperature process map: <op, <device, channel>>
    const std::map<uint16_t, std::shared_ptr<Tmp100<PsI2c>>> temp_instr_map = 
        {
          { TEMP1, tmp100_0_ },
          { TEMP2, tmp100_1_ },
          { TEMP3, tmp100_2_ }
        };


    // QueueSetHandle_t resp_queue_set;



    void latch_conf();
    void update_loads( char* loads );
    void send_spi_bit( int chip_sel, int val );
    void load_ad9252reg( int chip_sel, int addr, int data );
    int  ad9252_cfg( int chip_num, int addr, int data );
    void zddm_arm( int mode, int val );

    void rx_msg_proc(const typename GermaniumNetwork<GermaniumDetector>::UdpRxMsg& udp_msg);
    //void tx_msg_proc();

    void ps_i2c_access_task();
    void ps_xadc_access_task();

public:
    static constexpr size_t REGISTER_SINGLE_ACCESS_REQ_QUEUE_LENG = 100;

    //struct RegisterSingleAccessReqStruct
    //{
    //    uint16_t op;
    //    uint32_t reg_addr;
    //};
    //using RegisterSingleAccessReq = RegisterSingleAccessReqStruct;

    GermaniumDetector();
    //void do_task_init();
};
