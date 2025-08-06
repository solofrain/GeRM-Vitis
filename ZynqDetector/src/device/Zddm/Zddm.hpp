#pragma once

#include "FreeRTOS.h"

#include "Register.hpp"
#include "task_wrap.hpp"

#include "queue.hpp"

//struct __attribute__((__packed__)) ZddmArmStruct
//{
//    uint16_t  mode;
//    uint16_t  val;
//};
//using ZddmArm = ZddmArmStruct;
//using ZddmAccessReq = ZddmArmStruct;

template<typename DerivedNetwork>
class Zddm
{
public:
    Zddm( Register&            reg_
        , QueueHandle_t const  zddm_access_req_queue );

    void create_device_access_tasks();

private:
    Register&            reg_;
    QueueHandle_t const  req_queue_;

    static constexpr UBaseType_t TASK_PRIORITY = 6;
    static constexpr uint32_t    TASK_STACK_SIZE = 1000;
    StaticTask_t                 task_tcb;
    StackType_t                  task_stack[TASK_STACK_SIZE];


    void zddm_arm( int mode, int val );
    void zddm_cfg_task();
};

#include "Zddm.tpp"
