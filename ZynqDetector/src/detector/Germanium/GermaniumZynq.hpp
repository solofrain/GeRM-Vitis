#pragma once

#include <atomic>
#include <map>
#include <vector>
#include <string>
#include <memory>

#include "FreeRTOS.h"
#include "semphr.h"

#include "PSI2C.hpp"
#include "PSXADC.hpp"
#include "PLInterface.hpp"

#define __FREERTOS__
//#define __LINUX__

#define REG_BASE_ADDR  0x43C00000
#define XADC_ADDR 0xF8007100

#define REG_VER        0x0





//=========================================
// Zynq class
//=========================================
template < typename Owner >
class GermaniumZynq : public Zynq<Owner>
{
protected:
    //Register reg_;
    std::shared_ptr<PSI2C>   psi2c0_;
    std::shared_ptr<PSI2C>   psi2c1_;
    std::shared_ptr<PSXADC>  psxadc_;

    

public:
    GermaniumZynq( uintptr_t base_addr
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

    auto add_ps_i2c( uint8_t bus_index );

    //PLI2CInterface* get_pl_i2c_interface( const std::string& name );
    //PLSPIInterface* get_pl_spi_interface( const std::string& name );
};
//=========================================
