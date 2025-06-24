#pragma once

#include <cstdint>

#include "FreeRTOS.h"

//=========================================
// PL Interface class
//=========================================
template <typename Owner>
class PLInterface
{
protected:
    Register<Owner> reg_;
    uint32_t config_reg_;
    uint32_t instr_reg_;
    uint32_t wr_data_reg_;
    uint32_t rd_data_reg_;
    uint32_t baud_rate_reg_;
    uint32_t baud_rate_;
    TaskHandle_t task_handle_;

    xSemaphoreHandle mutex_;

    
public:
    PLInterface( Register<Owner>& reg, uint32_t config_reg, uint32_t instr_reg, uint32_t data_reg, uint32_t baud_rate );

    void write( uint32_t instr, uint32_t data );
    uint32_t read( uint32_t instr, uint32_t data );
    void set_baud_rate( uint32_t baud_rate );
    void wait_for_completion();
};
//=========================================

//=========================================
// I2C Interface class
//=========================================
template <typename Owner>
class PLI2CInterface : public PLInterface<Owner>
{
public:
    using PLInterface<Owner>::PLInterface;
};
//=========================================

//=========================================
// SPI Interface class
//=========================================
template <typename Owner>
class PLSPIInterface : public PLInterface<Owner>
{
public:
    using PLInterface<Owner>::PLInterface;
};
//=========================================
