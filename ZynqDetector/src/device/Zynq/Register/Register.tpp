#include <cstdint>
#include <functional>

#include "FreeRTOS.h"

#include "task_wrap.hpp"


//=========================================
// Register class
//=========================================
Register::Register
    (
      uintptr_t base_addr
    , QueueHandle_t& register_single_access_req_queue
    , QueueHandle_t& register_single_access_resp_queue
    )
    : base_addr_ ( static_cast<uintptr_t>( base_addr ) )
    , register_single_access_req_queue_ ( register_single_access_req_queue )
    , register_single_access_resp_queue_ ( register_single_access_resp_queue )
{}

void Register::write( uint32_t offset, uint32_t value )
{
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        *(volatile uint32_t*)(base_addr_ + offset/4) = value;
        xSemaphoreGive( mutex_ );
    }
}
    
uint32_t Register::read( uint32_t offset )
{
    uint32_t value = 0;
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        value = *(volatile uint32_t*)(base_addr_ + offset/4);
        xSemaphoreGive( mutex_ );
    }
    return value;
}

void Register::set_status( uint32_t status )
{
    write( 0xC, status );
}

void Register::multi_access_start()
{
    while( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdFALSE );
}

void Register::multi_access_write( uint32_t offset, uint32_t value )
{
    *(volatile uint32_t*)(base_addr_ + offset/4) = value;
}
    
uint32_t Register::multi_access_read( uint32_t offset )
{
    return *(volatile uint32_t*)(base_addr_ + offset/4);
}

void Register::multi_access_end()
{
    xSemaphoreGive( mutex_ );
}


void Register::task()
{
    RegisterSingleAccessReq  req;
    RegisterSingleAccessResp resp;

    uint32_t offset;

    //auto param = static_cast<reg_single_access_task_param_t*>(pvParameters);

    while(1)
    {
        xQueueReceive( register_single_access_req_queue_
                     , &req
					 , portMAX_DELAY );
        
        offset = static_cast<uint32_t>( req.op & 0x7fff );

        if ( (req.op & 0x8000) != 0 )
        {
            resp.data = read( offset );
            resp.op = req.op;
            xQueueSend( register_single_access_resp_queue_
                      , &resp
                      , 0UL
                      );
        }
        else
        {
            write( req.data, offset );
        }
    }
}

void Register::create_register_single_access_task()
{
    auto task_func = std::make_unique<std::function<void()>>([this]() { task(); });
    xTaskCreate( task_wrapper, "Register Single Access", 1000, &task_func, 1, NULL );
}
//=========================================
