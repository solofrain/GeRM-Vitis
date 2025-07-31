// C++ includes
#include <iterator>
#include <portmacro.h>
#include <memory>

// FreeRTOS includes
#include "FreeRTOS.h"
//#include "msg.hpp"
#include "task.h"
#include "queue.h"
#include "timers.h"

// Xilinx includes
#include "xil_printf.h"
#include "xparameters.h"

// Project includes
#include "ZynqDetector.hpp"
#include "GermaniumDetector.hpp"
#include "GermaniumZynq.hpp"
#include "GermaniumNetwork.hpp"

#define TIMER_ID 1
#define DELAY_10_SECONDS 10000UL
#define DELAY_1_SECOND 1000UL
#define TIMER_CHECK_THRESHOLD 9
/*-----------------------------------------------------------*/


static TimerHandle_t xPollTimer = NULL;


#if (configSUPPORT_STATIC_ALLOCATION == 1)
#define QUEUE_BUFFER_SIZE 100

uint8_t ucQueueStorageArea[QUEUE_BUFFER_SIZE];
StackType_t xStack1[configMINIMAL_STACK_SIZE];
StackType_t xStack2[configMINIMAL_STACK_SIZE];
StaticTask_t xTxBuffer, xRxBuffer;
StaticTimer_t xTimerBuffer;
static StaticQueue_t xStaticQueue;
#endif

GermaniumDetector::GermaniumDetector()
    : ZynqDetector< GermaniumDetector
                  , GermaniumNetwork
                  , GermaniumZynq
                  >()
    , ltc2309_0_ ( std::make_unique<Ltc2309<PsI2c>>( /*psi2c_1_
                                                   , */LTC2309_0_I2C_ADDR
                                                   , true
                                                   , psi2c_1_access_req_queue_
                                                   , chan_assign_
                                                   , this->logger_
                                                   )
                 )
    , ltc2309_1_ ( std::make_unique<Ltc2309<PsI2c>>( /*psi2c_1_
                                                   , */LTC2309_1_I2C_ADDR
                                                   , true
                                                   , psi2c_1_access_req_queue_
                                                   , chan_assign_
                                                   , this->logger_
                                                   )
                 )
    , dac7678_   ( std::make_unique<Dac7678<PsI2c>>( /*psi2c_1_
                                                   , */DAC7678_I2C_ADDR
                                                   , psi2c_1_access_req_queue_
                                                   , chan_assign_
                                                   , this->logger_
                                                   )
                 )
    , tmp100_0_  ( std::make_unique<Tmp100<PsI2c>>( /*psi2c_0_
                                                  , */TMP100_0_I2C_ADDR
                                                  , psi2c_0_access_req_queue_
                                                  , this->logger_
                                                  )
                 )
    , tmp100_1_  ( std::make_unique<Tmp100<PsI2c>>( /*psi2c_0_
                                                  , */TMP100_1_I2C_ADDR
                                                  , psi2c_0_access_req_queue_
                                                  , this->logger_
                                                  )
                 )
    , tmp100_2_  ( std::make_unique<Tmp100<PsI2c>>( /*psi2c_0_
                                                  , */TMP100_2_I2C_ADDR
                                                  , psi2c_0_access_req_queue_
                                                  , this->logger_
                                                  )
                 )
    , ad9252_    ( std::make_unique<Ad9252>( *this->zynq_->base_->reg_
                                           , ad9252_access_req_queue_
                                           )
                 )
{

    psi2c_0_access_req_queue_ = xQueueCreate( 5, sizeof(PsI2cAccessReq) );
    psi2c_1_access_req_queue_ = xQueueCreate( 5, sizeof(PsI2cAccessReq) );
    psxadc_access_req_queue_  = xQueueCreate( 5, sizeof(PsXadcAccessReq) );
    ad9252_access_req_queue_  = xQueueCreate( 3, sizeof(Ad9252AccessReq) );

    psi2c_access_resp_queue_ = xQueueCreate( 10, sizeof(PsI2cAccessResp) );
    //psi2c_1_access_resp_queue_ = xQueueCreate( 5, sizeof(PsI2cAccessResp) );
    psxadc_access_resp_queue_  = xQueueCreate( 5, sizeof(PsXadcAccessResp) );

    //resp_queue_set = xQueueCreateSet(50);

    //xQueueAddToSet( psi2c_0_resp_queue, resp_queue_set );
    //xQueueAddToSet( psi2c_1_resp_queue, resp_queue_set );
    //xQueueAddToSet( psxadc_resp_queue, resp_queue_set );

    auto z = std::make_unique<GermaniumZynq>( register_single_access_req_queue_
                                            , register_single_access_resp_queue_
                                            , psi2c_0_access_req_queue_
                                            //, psi2c_0_access_resp_queue_
                                            , psi2c_1_access_req_queue_
                                            , psi2c_access_resp_queue_
                                            , psxadc_access_req_queue_
                                            , psxadc_access_resp_queue_
                                            , this->logger_
                                            );

    this->set_zynq ( std::move(z) );

    auto n = std::make_unique<GermaniumNetwork>( register_single_access_req_queue_
                                               , register_single_access_resp_queue_
                                               , psi2c_0_access_req_queue_
                                               //, psi2c_0_access_resp_queue_
                                               , psi2c_1_access_req_queue_
                                               , psi2c_access_resp_queue_
                                               , psxadc_access_req_queue_
                                               , psxadc_access_resp_queue_
                                               , ad9252_access_req_queue_
                                               , mars_access_req_queue_
                                               , zddm_access_req_queue_
                                               , this->logger_
                                               );

    this->set_network( std::move(n) );

    network_->network_init();
}
//===============================================================


//===============================================================
// Latch MARS configuration.
//===============================================================
//void GermaniumDetector::polling_task_init()
//{
//    poll_list.emplace_back( HV_RBV | 0x8000 );
//    //zynq_->reg_->= new GermaniumRegister();
//}

//===============================================================

//===============================================================
//  DETECTOR_TYPE
//===============================================================
/*void proc_detector_type(const UdpRxMsg& msg)
{
    if ( msg.op&0x8000 == 0 )
    {
        if ( msg.zynq_->reg_->cc_zynq_->reg_->ata == 0 )
        {
            nelm_ = 192;
            num_chips_ = 6;
        }
        else
        {
            nelm_ = 384;
            num_chips_ = 12;
        }
    }
    register_access_request_proc(msg);
}
//===============================================================
*/


//===============================================================
//===============================================================
//void GermaniumDetector::register_access_request_proc(const UdpRxMsg &msg)
//{
//    RegisterAccessRequest req;
//    req.op = msg.op;
//    xQueueSend( register_access_request_queue
//              , req
//              , 0UL
//              );
//}
//===============================================================


//===============================================================
// Only UDP tasks for GeDetector.
//===============================================================
//void GermaniumDetector::task_init()
//{
//    xTaskCreate( udp_rx_task
//               , (const char *)"UDP_RX"
//               , configMINIMAL_STACK_SIZE
//               , NULL
//               , tskIDLE_PRIORITY
//               , &udp_rx_task_handle_
//               );
//
//    xTaskCreate( udp_tx_task
//               , (const char *)"UDP_TX"
//               , configMINIMAL_STACK_SIZE
//               , NULL
//               , tskIDLE_PRIORITY + 1
//               , &udp_tx_task_handle_
//               );
//
//    xTaskCreate( psi2c_task
//               , ( const char* ) "PsI2c0"
//               , configMINIMAL_STACK_SIZE
//               , NULL
//               , tskIDLE_PRIORITY + 1
//               , &psi2c_0_task_handler_
//               );
//
//    xTaskCreate( psi2c_task
//               , ( const char* ) "PsI2c1"
//               , configMINIMAL_STACK_SIZE
//               , NULL
//               , tskIDLE_PRIORITY + 1
//               , &psi2c_1_task_handler_
//               );
//
//    xTaskCreate( psxadc_task
//               , ( const char* ) "PsXadc"
//               , configMINIMAL_STACK_SIZE
//               , NULL
//               , tskIDLE_PRIORITY + 1
//               , &psxadc_task_handler_
//               );
//}

void GermaniumDetector::create_device_access_tasks()
{
    // Devices managed by Zynq
    this->zynq_->base_->create_device_access_tasks();
    ad9252_->create_device_access_tasks();
    //this->zynq_->reg_->create_register_single_access_task();

//    psxadc_.create_psxadc_task();
//
//    psi2c_0_.create_psi2c_task();
//    psi2c_1_.create_psi2c_task();
//
//    xTaskCreate( register_multi_access_task_wrapper
//               , (const char *)"Register-Multi-Access"
//               , configMINIMAL_STACK_SIZE
//               , this
//               , tskIDLE_PRIORITY
//               , &register_multi_access_task_handle_ );
//
//    xTaskCreate( psxadc_.task_wrapper
//               , (const char *)"PsXadc-Access"
//               , configMINIMAL_STACK_SIZE
//               , &psxadc_
//               , tskIDLE_PRIORITY
//               , &psxadc_task_handle_ );
//
//    xTaskCreate( psi2c_0_.task_wrapper
//               , (const char *)"PsI2c-0-Access"
//               , configMINIMAL_STACK_SIZE
//               , &psi2c_0_
//               , tskIDLE_PRIORITY
//               , &psi2c_0_task_handle_ );
//
//    xTaskCreate( psi2c_1_.task_wrapper
//               , (const char *)"PsI2c-1-Access"
//               , configMINIMAL_STACK_SIZE
//               , &psi2c_1_
//               , tskIDLE_PRIORITY
//               , &psi2c_1_task_handle_ );
}
//===============================================================


////===============================================================
//// Register multi-access task wrapper
////===============================================================
//void GermaniumDetector::create_register_multi_access_task()
//{
//    auto task_func = std::make_unique<std::function<void()>>([this]() { register_multi_access_task(); });
//    xTaskCreate( task_wrapper, "Register Single Access", 1000, &task_func, 1, NULL );
//}
////===============================================================
//
//
////===============================================================
//// Register multi-access task
////===============================================================
//void GermaniumDetector::register_multi_access_task()
//{
//    GermaniumAccessReq req;
//
//    xQueueReceive( this->zynq_->base_->reg_->ulti_access_resp_queue_, &req, 0);
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
//            log_error( "Invalid op: %d", req.op & 0x7fff );
//    }
//}
////===============================================================


//===============================================================
// Germanium queue init
//===============================================================
//void GermaniumDetector::create_detector_queues()
//{
//    psi2c_0_req_queue = xQueueCreate( 5, sizeof(PsI2cAccessReq) );
//    psi2c_1_req_queue = xQueueCreate( 5, sizeof(PsI2cAccessReq) );
//    psxadc_req_queue  = xQueueCreate( 5, sizeof(PsXadcAccessReq) );
//
//    psi2c_0_resp_queue = xQueueCreate( 5, sizeof(PsI2cAccessResp) );
//    psi2c_1_resp_queue = xQueueCreate( 5, sizeof(PsI2cAccessResp) );
//    psxadc_resp_queue  = xQueueCreate( 5, sizeof(PsXadcAccessResp) );
//}
//===============================================================


//===============================================================
// Latch MARS configuration.
//===============================================================
//void GermaniumDetector::polling_task_init()
//{
//    poll_list.emplace_back( HV_RBV | 0x8000 );
//    poll_list.emplace_back( HV_CURR | 0x8000  );
//
//    TimerHandle_t xTimer = xTimerCreate( "1S Polling Tmer"
//        , pdMS_TO_TICKS(1000) // 1 second
//        , pdTRUE              // auto-reload
//        , ( void * ) 1        // timer ID
//        , polling_1s );
//}
////===============================================================
//
//
//void GermaniumDetector::polling_1s()
//{
//    UdpRxMsg msg;
//    
//    for ( auto iter : poll_list )
//    {
//        int instr = *iter;
//        auto it = instr_map_.find(instr);
//        if (it != instr_map_.end())
//        {
//            msg.op = instr;
//            it->second(msg);  // Call the corresponding function
//        }
//        else
//        {
//            std::cout << "Unknown instruction: " << instr << '\n';
//        }
//    }
//}
//===============================================================

//===============================================================
// Latch MARS configuration.
//===============================================================
void GermaniumDetector::latch_conf()
{
    this->zynq_->base_->reg_->write( GermaniumNetwork::MARS_CONF_LOAD, 2 );
    this->zynq_->base_->reg_->write( GermaniumNetwork::MARS_CONF_LOAD, 0 );
}
//===============================================================


//===============================================================
// Stuff MARS.
//===============================================================
void GermaniumDetector::stuff_mars()
{
    for ( int i = 0; i < 12; i++ )
    {
        this->zynq_->base_->reg_->write( GermaniumNetwork::MARS_CONF_LOAD, 4 );
        this->zynq_->base_->reg_->write( GermaniumNetwork::MARS_CONF_LOAD, 0 );

        for ( int j = 0; j < 14; j++ )
        {
            this->zynq_->base_->reg_->write( GermaniumNetwork::MARS_CONF_LOAD, loads_[i][j] );
            latch_conf();
            vTaskDelay(pdMS_TO_TICKS(1));;
        }

        this->zynq_->base_->reg_->write( GermaniumNetwork::MARS_CONF_LOAD, 0x00010000 << i );
        this->zynq_->base_->reg_->write( GermaniumNetwork::MARS_CONF_LOAD, 0 );
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
//===============================================================


//===============================================================
// Update loads.
//===============================================================
void GermaniumDetector::update_loads( char* loads )
{
    memcpy( loads_, loads, sizeof(loads_) );
}

void GermaniumDetector::send_spi_bit( int chip_sel, int val )
{
    sda = val & 0x1;

    // set sclk low
    this->zynq_->base_->reg_->write( GermaniumNetwork::ADC_SPI, (chip_sel | 0) );

    // set data with clock low
    this->zynq_->base_->reg_->write( GermaniumNetwork::ADC_SPI, (chip_sel | sda) );

    // set clk high
    this->zynq_->base_->reg_->write( GermaniumNetwork::ADC_SPI, (chip_sel | 0x2 | sda) );

    // set clk low
    this->zynq_->base_->reg_->write( GermaniumNetwork::ADC_SPI, (chip_sel | sda) );

    // set data low
    this->zynq_->base_->reg_->write( GermaniumNetwork::ADC_SPI, (chip_sel | 0) );
}
//===============================================================


//===============================================================
// Load AD9252 registers.
//===============================================================
void GermaniumDetector::load_ad9252reg( int chip_sel, int addr, int data )
{
    int i, j, k;

    // small delay
    for (k = 0; k < 100; k++)
        ;

    // Read/Write bit
    send_spi_bit(chip_sel, 0);

    // W1=W0=0 (word length = 1 byte)
    for (i = 1; i >= 0; i--)
        send_spi_bit(chip_sel, 0);

    // address
    for (j = 12; j >= 0; j--)
        send_spi_bit(chip_sel, addr >> j);

    // data
    for (j = 7; j >= 0; j--)
        send_spi_bit(chip_sel, data >> j);

    // small delay
    for (k = 0; k < 100; k++)
        ;
    return (0);
}
//===============================================================



//===============================================================
// Arm.
//===============================================================
void GermaniumDetector::zddm_arm( int mode, int val )
{
    RegisterSingleAccessResp resp;

    if ( pscal->mode == 0 )
    {
        if ( val == 1 )
        {
            resp.data = this->zynq_->base_->reg_->read( GermaniumNetwork::FRAME_NO );
            this->zynq_->base_->reg_->write( GermaniumNetwork::TRIG, val );
        }
        if ( val == 0 )
        {
            static_cast<GermaniumZynq*>(this->zynq_.get())->base_->reg_->write( GermaniumNetwork::TRIG, val );
        }
    }

    if ( pscal->mode == 1 )
    {
        if ( val == 1 )
        {
            RegisterSingleAccessResp resp;
            resp.data = this->zynq_->base_->reg_->read( GermaniumNetwork::FRAME_NO );
        }
        if ( val == 0 )
        {
            this->zynq_->base_->reg_->write( GermaniumNetwork::TRIG, val );
        }
    }
}
//===============================================================
