#pragma once

#include <atomic>
#include <map>
#include <vector>
#include <string>
#include <memory>

#include "FreeRTOS.h"
#include "semphr.h"

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
class GermaniumZynq : public Zynq<GermaniumZynq, GermaniumRegister, GermaniumDetector>
{
protected:
    //Register reg_;
    std::unique_ptr<PsI2c>   psi2c0_;
    std::unique_ptr<PsI2c>   psi2c1_;
    std::unique_ptr<PsXadc>  psxadc_;

    //static constexpr uintptr_t REG_BASE_ADDR  = 0x43C00000;
    //static constexpr uintptr_t PSI2C0_ADDR    = 0x43C00000;
    //static constexpr uintptr_t PSI2C1_ADDR    = 0x43C00000;
    //static constexpr uintptr_t XADC_ADDR      = 0xF8007100;
    //static constexpr uint32_t  PSI2C_CLK_FREQ = 0xF8007100;


public:
    GermaniumZynq( const QueueHandle_t register_single_access_req_queue
                 , const QueueHandle_t register_single_access_resp_queue
                 , const QueueHandle_t psi2c0_req_queue
                 , const QueueHandle_t psi2c0_resp_queue
                 , const QueueHandle_t psi2c1_req_queue
                 , const QueueHandle_t psi2c1_resp_queue
                 , const QueueHandle_t psxadc_req_queue
                 , const QueueHandle_t psxadc_resp_queue
                 );

//    GermaniumZynq( uintptr_t base_addr );

    //auto add_pl_i2c( const std::string& name, uint32_t instr_reg, uint32_t data_reg );
    //auto add_pl_spi( const std::string& name, uint32_t instr_reg, uint32_t data_reg );

    //auto add_ps_i2c( uint8_t bus_index );

    //PlI2cInterface* get_pl_i2c_interface( const std::string& name );
    //PlSpiInterface* get_pl_spi_interface( const std::string& name );
};
//=========================================
