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
                                    , multi_access_req_queue  ( multi_access_req_queue  )
                                    , multi_access_resp_queue ( multi_access_resp_queue )
{}


