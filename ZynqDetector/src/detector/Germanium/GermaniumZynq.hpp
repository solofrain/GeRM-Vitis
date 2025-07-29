#pragma once

#include <atomic>
#include <map>
#include <vector>
#include <string>
#include <memory>

#include "FreeRTOS.h"
#include "semphr.h"

#include "Logger.hpp"

#include "PsI2c.hpp"
#include "PsXadc.hpp"
//#include "PlInterface.hpp"

#define __FREERTOS__
//#define __LINUX__

#define REG_VER        0x0



class GermaniumDetector;
class GermaniumRegister;

//=========================================
// Zynq class
//=========================================
class GermaniumZynq : public Zynq<GermaniumZynq, GermaniumRegister>
{

public:
    GermaniumZynq( const QueueHandle_t              register_single_access_req_queue
                 , const QueueHandle_t              register_single_access_resp_queue
                 , const QueueHandle_t              register_multi_access_req_queue
                 , const QueueHandle_t              register_multi_access_resp_queue
                 , const QueueHandle_t              psi2c0_req_queue
                 //, const QueueHandle_t              psi2c0_resp_queue
                 , const QueueHandle_t              psi2c1_req_queue
                 , const QueueHandle_t              psi2c_resp_queue
                 //, const QueueHandle_t              psi2c1_resp_queue
                 , const QueueHandle_t              psxadc_req_queue
                 , const QueueHandle_t              psxadc_resp_queue
                 , const Logger<GermaniumRegister>& logger
                 );

    Zynq<GermaniumZynq, GermaniumRegister>* base_;

//    GermaniumZynq( uintptr_t base_addr );

    //auto add_pl_i2c( const std::string& name, uint32_t instr_reg, uint32_t data_reg );
    //auto add_pl_spi( const std::string& name, uint32_t instr_reg, uint32_t data_reg );

    //auto add_ps_i2c( uint8_t bus_index );

    //PlI2cInterface* get_pl_i2c_interface( const std::string& name );
    //PlSpiInterface* get_pl_spi_interface( const std::string& name );

    void create_device_access_tasks_special();

protected:
    //Register reg_;
    std::unique_ptr<PsI2c<GermaniumRegister>>     psi2c0_;
    std::unique_ptr<PsI2c<GermaniumRegister>>     psi2c1_;
    std::unique_ptr<PsXadc<GermaniumRegister>>    psxadc_;
    const Logger<GermaniumRegister>&              logger_;

    //static constexpr uintptr_t REG_BASE_ADDR  = 0x43C00000;
    //static constexpr uintptr_t PSI2C0_ADDR    = 0x43C00000;
    //static constexpr uintptr_t PSI2C1_ADDR    = 0x43C00000;
    //static constexpr uintptr_t XADC_ADDR      = 0xF8007100;
    //static constexpr uint32_t  PSI2C_CLK_FREQ = 0xF8007100;

    void device_access_tasks();

};
//=========================================
