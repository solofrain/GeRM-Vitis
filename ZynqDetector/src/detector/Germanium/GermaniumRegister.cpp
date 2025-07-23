#include "GermaniumRegister.hpp"

GermaniumRegister::GermaniumRegister( uintptr_t            base_addr
                                    , const QueueHandle_t  single_access_req_queue
                                    , const QueueHandle_t  single_access_resp_queue
                                    )
                                    : Register<GermaniumRegister> ( base_addr
                                                                  , single_access_req_queue
                                                                  , single_access_resp_queue
                                                                  )
{}


