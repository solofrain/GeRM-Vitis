#include "Logger.hpp"
#include "GermaniumRegister.hpp"

GermaniumRegister::GermaniumRegister( uintptr_t                        base_addr
                                    , const QueueHandle_t              single_access_req_queue
                                    , const QueueHandle_t              single_access_resp_queue
                                    , const QueueHandle_t              multi_access_req_queue
                                    , const QueueHandle_t              multi_access_resp_queue
                                    , const Logger<GermaniumRegister>& logger
                                    )
                                    : Register<GermaniumRegister> ( base_addr
                                                                  , single_access_req_queue
                                                                  , single_access_resp_queue
                                                                  , logger
                                                                  )
                                    , base_ ( static_cast<Register<GermaniumRegister>*>(this))
                                    , multi_access_req_queue_  ( multi_access_req_queue  )
                                    , multi_access_resp_queue_ ( multi_access_resp_queue )
{}


void GermaniumRegister::create_register_access_tasks_special()
{
    create_register_multi_access_task();
}

//===============================================================
// Register multi-access task wrapper
//===============================================================
void GermaniumRegister::create_register_multi_access_task()
{
    auto task_func = std::make_unique<std::function<void()>>([this]() { register_multi_access_task(); });
    xTaskCreate( task_wrapper, "Register Multi Access", 1000, &task_func, 1, NULL );
}
//===============================================================


////===============================================================
//// Register multi-access task
////===============================================================
//void GermaniumRegister::register_multi_access_task()
//{
//    RegisterMultiAccessReq req;
//
//    xQueueReceive( multi_access_resp_queue_, &req, 0);
//    switch ( req.op & 0x7fff )
//    {
//        case UPDATE_LOADS:
//            update_loads( &req.data )
//            break;
//
//        case STUFF_MARS:
//            stuff_mars();
//            break;
//
//        case AD9252_CNFG:
//            ad9252_cnfg( req.ad9252_cnfg.chip_num
//                       , req.ad9252_cnfg.addr
//                       , req.ad9252_cnfg.val );
//
//        case ZDDM_ARM:
//            zddm_arm( req.zddm_arm.mode
//                    , req.zddm_arm.val );
//            break;
//
//        default:
//            logger_.log_error( "Invalid op: %d", req.op & 0x7fff );
//    }
//}
//===============================================================

