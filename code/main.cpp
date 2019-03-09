#include "hwlib.hpp"

#include <can.hpp>

int main(void) {
  // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
    hwlib::wait_ms(1000);

    using can = r2d2::can_bus::controller<r2d2::can_bus::can0>;

    can::init();

    for (;;){
        
        hwlib::wait_ms(1000);
    }
}