//#include <atomic>
//#include <chrono>
//#include <thread>

#ifdef __LINUX__
#include <sys/mman.h>
#include <fcntl.h>  // For O_* constants
#include <unistd.h> // For close()
#endif

#include "xparameters.h"

#include "Register.hpp"
#include "Zynq.hpp"
#include "GermaniumZynq.hpp"
#include "GermaniumRegister.hpp"


//###################################################
// Definitions for FreeRTOS
//###################################################
#ifdef __FREERTOS__


//-----------------------------------------
GermaniumZynq::GermaniumZynq
    ( const QueueHandle_t              register_single_access_req_queue
    , const QueueHandle_t              register_single_access_resp_queue
    , const QueueHandle_t              register_multi_access_req_queue
    , const QueueHandle_t              register_multi_access_resp_queue
    , const QueueHandle_t              psi2c0_req_queue
    , const QueueHandle_t              psi2c0_resp_queue
    , const QueueHandle_t              psi2c1_req_queue
    , const QueueHandle_t              psi2c1_resp_queue
    , const QueueHandle_t              psxadc_req_queue
    , const QueueHandle_t              psxadc_resp_queue
    , const Logger<GermaniumRegister>& logger
    )
    : Zynq< GermaniumZynq
          , GermaniumRegister
          >
          ( XPAR_IOBUS_0_BASEADDR
          , register_single_access_req_queue
          , register_single_access_resp_queue
          , logger
          )
    , base_  ( static_cast<Zynq<GermaniumZynq, GermaniumRegister>*>(this) )
    , psi2c0_( std::make_unique<PsI2c<GermaniumRegister>>( 0
                                      , "psi2c0"
                                      , XPAR_I2C0_BASEADDR
                                      , XPAR_I2C0_CLOCK_FREQ
                                      , psi2c0_req_queue
                                      , psi2c0_resp_queue
                                      , logger
                                      )
             )
    , psi2c1_( std::make_unique<PsI2c<GermaniumRegister>>( 1
                                      , "psi2c1"
                                      , XPAR_I2C1_BASEADDR
                                      , XPAR_I2C1_CLOCK_FREQ
                                      , psi2c1_req_queue
                                      , psi2c1_resp_queue
                                      , logger
                                      )
             )
    , psxadc_( std::make_unique<PsXadc<GermaniumRegister>>( "psxadc"
                                       , psxadc_req_queue
                                       , psxadc_resp_queue
                                       , logger
                                       )
             )
    , logger_ ( logger )
{
    auto r = std::make_unique<GermaniumRegister>( XPAR_IOBUS_0_BASEADDR
                                                , register_single_access_req_queue
                                                , register_single_access_resp_queue
                                                , register_multi_access_req_queue
                                                , register_multi_access_resp_queue
                                                , logger
                                                );
    this->set_register( std::move(r) );
}



void GermaniumZynq::create_device_access_tasks_special()
{
    psi2c0_->create_psi2c_task();
    psi2c1_->create_psi2c_task();
    psxadc_->create_psxadc_task();
}

/*
auto Zynq::add_pl_i2c( const std::string& name
                     , uint32_t instr_reg
                     , uint32_t wr_data_reg
                     , uint32_t rd_data_reg )
{
    pl_i2cs_.emplace_back( std::piecewise_construct,
                      std::forward_as_tuple( name ),
                      std::forward_as_tuple( reg_, instr_reg, wr_data_reg, rd_data_reg ) );
}

void Zynq::add_pl_spi( const std::string& name
                     , uint32_t instr_reg
                     , uint32_t wr_data_reg
                     , uint32_t rd_data_reg )
{
    pl_spis_.emplace( std::piecewise_construct
                    , std::forward_as_tuple( name )
                    , std::forward_as_tuple( reg_, instr_reg, wr_data_reg, rd_data_reg ) );
}
*/
//auto GermaniumZynq::add_psi2c_interface( uint32_t bus_index )
//{
//    return ps_i2cs_.emplace_back( bus_index );
//}

//I2cInterface* Zynq::get_pli2c_interface( const std::string& name )
//{
//    auto it = pl_i2c_interfaces_.find( name );
//    return ( it !=pl_ i2c_interfaces_.end() ) ? &(it->second) : nullptr;
//}
//
//SpiInterface* Zynq::get_plspi_interface(const std::string& name)
//{
//    auto it = pl_spi_interfaces_.find( name );
//    return ( it != pl_spi_interfaces_.end() ) ? &(it->second) : nullptr;
//}

//=========================================
#endif



#ifdef __LINUX__
//=========================================
// Definitions for Linux
//=========================================
Zynq::Zynq(uint32_t axi_base_addr)
    : axi_base_addr( axi_base_addr )
    , reg_         ( nullptr       )
    , reg_size     ( 0x10000       )
{
    int fd = open("/dev/mem",O_RDWR | O_SYNC);
    if (fd == -1)
    {
        throw std::runtime_error("Failed to open /dev/mem. Try root.");
    }

    try
    {
        //reg_size = getpagesize();
        reg_ = static_cast<uint32_t *>( mmap( nullptr
                                            , reg_size
                                            , PROT_READ | PROT_WRITE
                                            , MAP_SHARED
                                            , fd
                                            , axi_base_addr
                                            )
                                      );
    }
    catch (const std::exception& e)
    {
        close(fd);
        throw std::runtime_error( "Memory mapping failed: " + std::string( e.what())  );
    }


    close(fd);
    
    if(reg == MAP_FAILED)
    {
        throw std::runtime_error( "Memory mapping failed" );
    }

    trace_reg( __func__
             , ": Zynq object created at 0x"
             , std::hex, static_cast<void*>(reg), std::dec );
}

//-----------------------------------------

Zynq::~Zynq()
{
    if (reg != nullptr)
    {
        munmap( reg, reg_size );
    }
    trace_reg( __func__, ": Zynq object destructed." );
}

//-----------------------------------------

void Zynq::reg_wr( size_t offset, uint32_t value )
{
    if (offset % sizeof(uint32_t) != 0) {
        throw std::runtime_error("Offset must be aligned to register size");
    }

    volatile uint32_t *registerPtr = reg + offset / sizeof(uint32_t);
    while( reg_lock.exchange( true, std::memory_order_acquire) );
    *registerPtr = value;
    reg_lock.store( false, std::memory_order_release );


    trace_reg( __func__,
               ": write 0x", std::hex, value,
               " to 0x", offset,
               " (0x",
               static_cast<void*>(const_cast<uint32_t*>(registerPtr)),
               std::dec
             );

    reg_rd(offset);
}

//-----------------------------------------

uint32_t Zynq::reg_rd(size_t offset)
{
    uint32_t val;
    if (offset % sizeof(uint32_t) != 0)
    {
        throw std::runtime_error("Offset must be aligned to register size");
    }

    volatile uint32_t *registerPtr = reg + offset / sizeof(uint32_t);
    while( reg_lock.exchange( true, std::memory_order_acquire) );
    val = *registerPtr;
    reg_lock.store( false, std::memory_order_release );

    trace_reg( __func__, ": read 0x",
               std::hex, val,
               " @ 0x", offset,
               " (0x",
               static_cast<void*>(const_cast<uint32_t*>(registerPtr)),
               std::dec
             );

    //io_wait();
    return val;
}

//=========================================
#endif
