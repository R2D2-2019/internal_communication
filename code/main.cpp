#include "hwlib.hpp"

#include <can.hpp>

int main(void) {
    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
    hwlib::wait_ms(1000);

    hwlib::cout << "Starting Due\n";

    using can = r2d2::can_bus::controller<r2d2::can_bus::can0>;

    hwlib::cout << "Correct init: " << can::init<r2d2::can_bus::baudrate::BPS_250K>() << '\n';

    for(uint8_t i = 0; i < 4; i++){
        r2d2::can_bus::detail::_set_mailbox_filter<r2d2::can_bus::can0>(i, 0, 0);
    }
    
    for (;;){
        for(uint8_t i = 0; i < 4; i++){
            if(!r2d2::can_bus::detail::_mailbox_rx_stores<r2d2::can_bus::can0>::buffers[i].empty()){
                hwlib::cout << "Got data buffer:" << i << "\n";
                while(!r2d2::can_bus::detail::_mailbox_rx_stores<r2d2::can_bus::can0>::buffers[i].empty()){
                    auto data = r2d2::can_bus::detail::_mailbox_rx_stores<r2d2::can_bus::can0>::buffers[i].copy_and_pop();
                    hwlib::cout << hwlib::hex << data.data.low << ", ";
                }
                hwlib::cout << "\n";
            }
        }
        hwlib::wait_ms(10000);
    }
}