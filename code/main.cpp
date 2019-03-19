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

    auto led = hwlib::target::pin_out(hwlib::target::pins::d6);

    uint32_t start = hwlib::now_us();
    uint32_t counter = 0;
    uint32_t ticks = 0;

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

            counter += 1;
            const auto pressed = frame.as_frame_type<frame_type::BUTTON_STATE>().pressed;
            led.write(pressed);

            ticks = hwlib::now_us() - start;
            if (ticks >= 1'000'000) {
                hwlib::cout << counter << "\r\n";
                start = hwlib::now_us();
                ticks = 0;
                counter = 0;
            }   
        }
    }
}