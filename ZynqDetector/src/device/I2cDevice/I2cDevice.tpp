#include "FreeRTOS.h"


//============================================
// I2cDevice constructor.
//============================================
template< typename I2cType >
I2cDevice<I2cType>::I2cDevice( /*const I2cType&                     i2c
                             , */uint8_t                            i2c_addr
                             , const QueueHandle_t                req_queue
                             , const Logger&                      logger
                             )
//   requires IsSameType<T, PsI2c>
                               : /*i2c_         ( i2c                                )
                               , */i2c_addr_    ( i2c_addr                           )
                               , req_queue_   ( req_queue                          )
                               //, chan_assign_ ( chan_assign                        )
                               , logger_      ( logger                             )
{}

