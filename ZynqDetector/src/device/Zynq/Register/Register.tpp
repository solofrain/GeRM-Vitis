#include <cstdint>
#include <functional>

#include "FreeRTOS.h"

#include "task_wrap.hpp"


//=========================================
// Register class
//=========================================
template< typename DerivedRegister >
Register<DerivedRegister>::Register
    (
      uintptr_t base_addr
    , QueueHandle_t& single_access_req_queue
    , QueueHandle_t& single_access_resp_queue
    )
    : base_addr_ ( static_cast<uintptr_t>( base_addr ) )
    , single_access_req_queue_ ( single_access_req_queue )
    , single_access_resp_queue_ ( single_access_resp_queue )
{}

template< typename DerivedRegister >
void Register<DerivedRegister>::write( uint32_t offset, uint32_t value )
{
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        *(volatile uint32_t*)(base_addr_ + offset/4) = value;
        xSemaphoreGive( mutex_ );
    }
}
    
template< typename DerivedRegister >
uint32_t Register<DerivedRegister>::read( uint32_t offset )
{
    uint32_t value = 0;
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        value = *(volatile uint32_t*)(base_addr_ + offset/4);
        xSemaphoreGive( mutex_ );
    }
    return value;
}

template< typename DerivedRegister >
void Register<DerivedRegister>::set_status( uint32_t status )
{
    write( 0xC, status );
}

template< typename DerivedRegister >
void Register<DerivedRegister>::multi_access_start()
{
    while( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdFALSE );
}

template< typename DerivedRegister >
void Register<DerivedRegister>::multi_access_write( uint32_t offset, uint32_t value )
{
    *(volatile uint32_t*)(base_addr_ + offset/4) = value;
}
    
template< typename DerivedRegister >
uint32_t Register<DerivedRegister>::multi_access_read( uint32_t offset )
{
    return *(volatile uint32_t*)(base_addr_ + offset/4);
}

template< typename DerivedRegister >
void Register<DerivedRegister>::multi_access_end()
{
    xSemaphoreGive( mutex_ );
}


template< typename DerivedRegister >
void Register<DerivedRegister>::single_access_task()
{
    RegisterSingleAccessReq  req;
    RegisterSingleAccessResp resp;

    uint32_t offset;

    //auto param = static_cast<reg_single_access_task_param_t*>(pvParameters);

    while(1)
    {
        xQueueReceive( single_access_req_queue_
                     , &req
					 , portMAX_DELAY );
        
        offset = static_cast<uint32_t>( req.op & 0x7fff );

        if ( (req.op & 0x8000) != 0 )
        {
            resp.data = read( offset );
            resp.op = req.op;
            xQueueSend( single_access_resp_queue_
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

template< typename DerivedRegister >
void Register<DerivedRegister>::create_register_single_access_task()
{
    auto task_func = std::make_unique<std::function<void()>>([this]() { single_access_task(); });
    xTaskCreate( task_wrapper, "Register Single Access", 1000, &task_func, 1, NULL );
}
//=========================================
