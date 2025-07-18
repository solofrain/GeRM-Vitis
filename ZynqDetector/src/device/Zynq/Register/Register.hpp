#pragma once

#include <cstdint>
#include <memory>

#include "FreeRTOS.h"
#include "semphr.h"

#include "queue.hpp"

class Register
{
public:

    Register( uintptr_t base_addr,
              QueueHandle_t& register_single_access_req_queue,
              QueueHandle_t& register_single_access_resp_queue );

    void write( uint32_t offset, uint32_t value );
    uint32_t read( uint32_t offset );

    void multi_access_start();
    void multi_access_write( uint32_t offset, uint32_t value );
    uint32_t multi_access_read( uint32_t offset );
    void multi_access_end();

    void set_status( uint32_t status );

    void create_register_single_access_task();

private:
    uintptr_t base_addr_;
    xSemaphoreHandle mutex_;
    QueueHandle_t& register_single_access_req_queue_;
    QueueHandle_t& register_single_access_resp_queue_;
    void task();
};

#include "Register.tpp"
