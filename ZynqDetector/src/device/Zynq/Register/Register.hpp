#pragma once

#include <cstdint>
#include <memory>

#include "semphr.h"
#include "FreeRTOS.h"

#include "queue.hpp"

template <typename Owner>
class Register
{
public:

    Register( uintptr_t base_addr, Owner* owner );

    void write( uint32_t offset, uint32_t value );
    uint32_t read( uint32_t offset );

    void multi_access_start();
    void multi_access_write( uint32_t offset, uint32_t value );
    uint32_t multi_access_read( uint32_t offset );
    void multi_access_end();

    void create_register_single_access_task();

private:
    uintptr_t base_addr_;
    xSemaphoreHandle mutex_;
    Owner* owner_;
    void task();
};

#include "Register.tpp"
