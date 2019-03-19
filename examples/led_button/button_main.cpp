#include "hwlib.hpp"

#include <comm.hpp>

int main() {
    using namespace r2d2;

    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;

    hwlib::wait_ms(10);

    r2d2::comm_c comm;

    std::array<frame_id, 8> frames = {
        r2d2::frame_type::BUTTON_STATE
    };

    comm.listen_for_frames(frames);

    auto pin_in = hwlib::target::pin_in(hwlib::target::pins::d53);
    pin_in.pullup_enable();

    for (;;) {
        while (comm.has_data()) {
            auto frame = comm.get_data();

            if (!frame.request) {
                continue;
            }

            if (frame.type != frame_type::BUTTON_STATE) {
                continue;
            }

            packet_button_state_s state;
            state.pressed = ! pin_in.read();
            comm.send(state);
        }
    }
}