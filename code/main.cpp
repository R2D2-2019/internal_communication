#include <comm.hpp>
#include "hwlib.hpp"


int main() {
    using namespace r2d2;

    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;

    hwlib::wait_ms(1000);

    r2d2::comm_c comm;
    r2d2::comm_c comm2;

    comm2.listen_for_frames({r2d2::frame_type::BUTTON_STATE});

    for (;;) {
        frame_button_state_s state;
        state.pressed = true;

        comm.send(state);

        while (comm2.has_data()) {
            const auto frame = comm2.get_data();

            hwlib::cout << "rec fr\n";
        }
    }
}