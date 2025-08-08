#pragma once

#include <string>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "xiicps.h"
#include "xparameters.h"

#include "Logger.hpp"
#include "task_wrap.hpp"
#include "queue.hpp"


class PsI2c
{
private:

    XIicPs_Config* i2cps_config_ptr_;
    XIicPs         i2c_ps_;

    uint8_t        bus_index_;
    std::string    name_;
    uint32_t       device_id_;
    uintptr_t      base_addr_;
    uint32_t       clk_freq_;
    QueueHandle_t  req_queue_;
    QueueHandle_t  resp_queue_;
    
    static constexpr UBaseType_t TASK_PRIORITY   = 2;
    static constexpr uint32_t    TASK_STACK_SIZE = 1000;
    StaticTask_t                 task_tcb;
    StackType_t                  task_stack[TASK_STACK_SIZE];

    StaticTask_t task_tcb_;
    StackType_t task_stack_[1000];  // match your xTaskCreate() stack size

    TaskConfig task_cfg_;


    xSemaphoreHandle mutex_;

    const Logger& logger_;

    int write( char* buffer, uint16_t length, uint16_t slave_address );
    int read( char* buffer, uint16_t length, uint16_t slave_address );

    void task();

public:
  
    PsI2c( const uint8_t                  bus_index
         , const std::string              name
         //, const uint32_t                 device_id
         , const uint32_t                 base_addr
         , const uint32_t                 clk_freq
         , const QueueHandle_t            req_queue
         , const QueueHandle_t            resp_queue
         , const Logger& logger
         );

    void create_psi2c_task();
};

