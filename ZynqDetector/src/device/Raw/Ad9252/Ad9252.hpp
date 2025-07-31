

#include "task_wrap.hpp"

class Ad9252 {

public:
    Ad9252( const Register&       reg_
          , const QueueHandle_t   ad9252_access_req_queue );

    void create_device_access_tasks();

private:
    const Register&        reg_;
    const QueueHandle_t    access_req_queue;

    void set_clk_skew( int chip_num, int skew );
    void ad9252_cnfg( int chip_num, int addr, int val );
    void load_reg( int chip_sel, int addr, int val );
    void send_spi_bit( int chip_sel, int val );

    void create_ad9252_cfg_task();

};

