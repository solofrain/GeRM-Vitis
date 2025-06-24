#include <cstdint>
#include <functional>

#include "FreeRTOS.h"

#include "task_wrap.hpp"


//=========================================
// Register class
//=========================================
template<typename Owner>
Register<Owner>::Register( uintptr_t base_addr, Owner* owner  )
{
    base_addr_ = reinterpret_cast<volatile uint32_t*>( base_addr );
    owner_ = owner;
}

template<typename Owner>
void Register<Owner>::write( uint32_t offset, uint32_t value )
{
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        *(volatile uint32_t*)(base_addr_ + offset/4) = value;
        xSemaphoreGive( mutex_ );
    }
}
    
template<typename Owner>
uint32_t Register<Owner>::read( uint32_t offset )
{
    uint32_t value = 0;
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        value = *(volatile uint32_t*)(base_addr_ + offset/4);
        xSemaphoreGive( mutex_ );
    }
    return value;
}

template<typename Owner>
void Register<Owner>::multi_access_start()
{
    while( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdFALSE );
}

template<typename Owner>
void Register<Owner>::multi_access_write( uint32_t offset, uint32_t value )
{
    *(volatile uint32_t*)(base_addr_ + offset/4) = value;
}
    
template<typename Owner>
uint32_t Register<Owner>::multi_access_read( uint32_t offset )
{
    return *(volatile uint32_t*)(base_addr_ + offset/4);
}

template<typename Owner>
void Register<Owner>::multi_access_end()
{
    xSemaphoreGive( mutex_ );
}


template<typename Owner>
void Register<Owner>::task()
{
    RegisterSingleAccessReq  req;
    RegisterSingleAccessResp resp;

    uint32_t offset;

    //auto param = static_cast<reg_single_access_task_param_t*>(pvParameters);

    while(1)
    {
        xQueueReceive( owner_.register_single_access_request_queue
                     , &req
					 , portMAX_DELAY );
        
        offset = static_cast<uint32_t>( req.op & 0x7fff );

        if ( (req.op & 0x8000) != 0 )
        {
            resp.data = read( offset );
            resp.op = req.op;
            xQueueSend( owner_.register_single_access_response_queue
                      , resp
                      , 0UL
                      );
        }
        else
        {
            write( req.data, offset );
        }
    }
}

template<typename Owner>
void Register<Owner>::create_register_single_access_task()
{
    auto task_func = std::make_unique<std::function<void()>>([this]() { task(); });
    xTaskCreate( task_wrapper, "Register Single Access", 1000, &task_func, 1, NULL );
}
//=========================================
