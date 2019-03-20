#include "hwlib.hpp"

#include <comm.hpp>

#include "module.hpp"

int main() {
    using namespace r2d2;

    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;

    hwlib::wait_ms(10);

    r2d2::comm_c comm;

    auto pin_in = hwlib::target::pin_in(
        hwlib::target::pins::d53
    );

    pin_in.pullup_enable();

    button::module_c module(comm, pin_in);

    for (;;) {
        module.process();
    }
}