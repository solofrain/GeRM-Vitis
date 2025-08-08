#pragma once

#include "FreeRTOS.h"
#include "task.h"


struct TaskConfig {
    void (*entry)(void*);
    void* context;
};

inline void task_wrapper(void* param)
{
    auto* cfg = static_cast<TaskConfig*>(param);
    cfg->entry(cfg->context);
}


//void task_wrapper(void* pvParameters);

