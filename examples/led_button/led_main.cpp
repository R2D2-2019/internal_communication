#include "hwlib.hpp"

#include <comm.hpp>

int main() {
    using namespace r2d2;

    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;

    hwlib::wait_ms(10);

    r2d2::comm_c comm;

    std::array<frame_id, 8> frames = {
        r2d2::frame_type::ACTIVITY_LED_STATE
    };

    comm.listen_for_frames(frames);

    auto led = hwlib::target::pin_out(hwlib::target::pins::d6);

    for (;;) {
        while (comm.has_data()) {
            auto frame = comm.get_data();

            if (frame.request) {
                continue;
            }

            if (frame.type != frame_type::ACTIVITY_LED_STATE) {
                continue;
            }

            const auto state = frame.as_frame_type<frame_type::ACTIVITY_LED_STATE>().state;
            led.write(state);
        }
    }
}