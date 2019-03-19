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

    for (;;) {
        comm.request(frame_type::BUTTON_STATE);

        while (comm.has_data()) {
            auto frame = comm.get_data();

            // This module doesn't handle requests
            if (frame.request) {
                continue;
            }

            if (frame.type != frame_type::BUTTON_STATE) {
                continue;
            }

            const auto pressed = frame.as_frame_type<frame_type::BUTTON_STATE>().pressed;
            
            packet_activity_led_state_s led_state;
            
            led_state.state = pressed;

            comm.send(led_state);
        }

        hwlib::wait_ms(10);
    }
}