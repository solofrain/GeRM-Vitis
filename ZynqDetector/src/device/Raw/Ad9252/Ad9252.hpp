

#include "task_wrap.hpp"

template<typename DerivedRegister>
class Ad9252 {

public:
    Ad9252( const DerivedRegister& reg_
          , const QueueHandle_t    ad9252_access_req_queue );

    void Ad9252::create_ad9252_cfg_task()

private:
    const DerivedRegister& reg_;
    const QueueHandle_t    access_req_queue;

    void set_clk_skew( int chip_num, int skew );
    void ad9252_cnfg( int chip_num, int addr, int val );
    void load_reg( int chip_sel, int addr, int val );
    void send_spi_bit( int chip_sel, int val );

    void create_device_access_tasks();

};

#include "Ad9252.tpp"
