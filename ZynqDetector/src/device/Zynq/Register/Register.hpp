#pragma once

#include <cstdint>
#include <memory>

#include "FreeRTOS.h"
#include "semphr.h"

#include "queue.hpp"
//#include "Logger.hpp"


struct RegisterSingleAccessReqStruct : AccessReq
{
    uint16_t op;
    uint32_t data;
};
using RegisterSingleAccessReq = RegisterSingleAccessReqStruct;


struct RegisterSingleAccessRespStruct : AccessResp
{
    uint16_t op;
    uint32_t  data;
};
using RegisterSingleAccessResp = RegisterSingleAccessRespStruct;

//-----------------------------

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
    void write( uint32_t offset, uint32_t value );
    uint32_t read( uint32_t offset );

    // Function set to perform multiple access
    void multi_access_start();
    void multi_access_write( uint32_t offset, uint32_t value );
    uint32_t multi_access_read( uint32_t offset );
    void multi_access_end();

    // Write to status register
    void set_status( uint32_t status );

    void create_register_access_tasks();
    void create_register_single_access_task();

    //const Register* base_;

private:
    uintptr_t base_addr_;
    xSemaphoreHandle  mutex_;
    QueueHandle_t     single_access_req_queue_;
    QueueHandle_t     single_access_resp_queue_;
    const Logger& logger_;

    void single_access_task();
};

#include "Register.tpp"
