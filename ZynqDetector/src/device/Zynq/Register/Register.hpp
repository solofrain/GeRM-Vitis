#pragma once

#include <cstdint>
#include <memory>

#include "FreeRTOS.h"
#include "semphr.h"

#include "queue.hpp"
//#include "Logger.hpp"

template<typename Reg> class Logger;

template <typename DerivedRegister>
class Register
{
public:

    Register( uintptr_t                        base_addr
            , const QueueHandle_t              single_access_req_queue
            , const QueueHandle_t              single_access_resp_queue
            , const Logger<DerivedRegister>&   logger
            );

    // Regular single access
    void write( uint32_t offset, uint32_t value );
    uint32_t read( uint32_t offset );

    // Function set to perform multiple access
    void multi_access_start();
    void multi_access_write( uint32_t offset, uint32_t value );
    uint32_t multi_access_read( uint32_t offset );
    void multi_access_end();

    // Write to status register
    void set_status( uint32_t status );

    void create_register_single_access_task();

    const Register<DerivedRegister>* base_;

private:
    uintptr_t base_addr_;
    xSemaphoreHandle  mutex_;
    QueueHandle_t     single_access_req_queue_;
    QueueHandle_t     single_access_resp_queue_;

    const Logger<DerivedRegister>& logger_;

    void single_access_task();
};

#include "Register.tpp"
