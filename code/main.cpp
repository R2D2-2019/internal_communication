#include <comm.hpp>
#include "hwlib.hpp"


int main() {
    using namespace r2d2;

    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;

    hwlib::wait_ms(1000);

    r2d2::comm_c comm;

    comm.listen_for_frames({frame_type::ALL});

    for (;;) {
        frame_display_filled_rectangle_s state;
        state.x = 0xAA;
        state.y = 0xBA;
        state.red = 0xFF;

        // comm.send_external({0xAA, 0xFA}, state);

        // hwlib::wait_ms(10);

        while(comm.has_data()){
            auto t = comm.get_data();

            const auto data = t.as_frame_type<
                frame_type::DISPLAY_FILLED_RECTANGLE>();

            hwlib::cout << "Got frame: " << int(data.x) << '\n';
        }
    }
}