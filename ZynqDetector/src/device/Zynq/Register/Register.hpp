#pragma once

#include <cstdint>
#include <memory>

#include "FreeRTOS.h"
#include "semphr.h"

#include "queue.hpp"
#include "task_wrap.hpp"

class Logger;

class Register
{
public:

    Register( uintptr_t             base_addr
            , const QueueHandle_t   single_access_req_queue
            , const QueueHandle_t   single_access_resp_queue
            , const Logger&         logger
            );

    // Regular single access
    void write( uint16_t offset, uint32_t value );
    uint32_t read( uint16_t offset );

    // Function set to perform multiple access
    void multi_access_start();
    void multi_access_write( uint16_t offset, uint32_t value );
    uint32_t multi_access_read( uint16_t offset );
    void multi_access_end();

    // Write to status register
    void set_status( uint32_t status );

    void create_register_access_tasks();

    //const Register* base_;

private:
    uintptr_t base_addr_;
    xSemaphoreHandle  mutex_;
    QueueHandle_t     single_access_req_queue_;
    QueueHandle_t     single_access_resp_queue_;
    const Logger& logger_;

    static constexpr UBaseType_t TASK_PRIORITY   = 5;
    static constexpr uint32_t    TASK_STACK_SIZE = 1000;
    StaticTask_t                 task_tcb_;
    StackType_t                  task_stack_[TASK_STACK_SIZE];
    TaskConfig                   task_cfg_;

    void create_register_single_access_task();
    void single_access_task();
};

