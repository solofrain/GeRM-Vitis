#pragma once
/**
 * @file I2cBus.hpp
 * @brief Base class of I2C bus.
 * 
 * This file declaures the base class of I2C bus.
 * 
 * @author
 * - Ji Li
 * 
 * @date
 * - Created: 
 * 
 * @copyright BNL
 * @license
 */

#include <string>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "xiicps.h"
#include "xparameters.h"

#inlcude "queue.hpp"


class I2cBus
{
protected:

    uint8_t           bus_index_;
    std::string       name_;

    xSemaphoreHandle  mutex_;

    static constexpr uint32_t TASK_STACK_SIZE = 1000;
    StaticTask_t              task_tcb;
    StackType_t               task_stack[TASK_STACK_SIZE];


    int write( char* buffer, uint16_t length, uint16_t slave_address );
    int read( char* buffer, uint16_t length, uint16_t slave_address );

    void task();

public:
    
    I2cBus( uint8_t bus_index
          , std::string name
          , QueueHandle_t req_queue
          , QueueHandle_t resp_queue );

    ~I2cBus() = default;

    void create_psi2c_task();
};
