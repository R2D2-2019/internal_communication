#include "hwlib.hpp"

#include <can.hpp>
#include <channel.hpp>

int main(void) {
  // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
    
    hwlib::wait_ms(10);

    using can_frame = r2d2::can_bus::detail::_can_frame_s;
    using bus = r2d2::can_bus::can0;
    using can = r2d2::can_bus::controller<bus>;
    using channel = r2d2::can_bus::channel_c<
        bus, r2d2::can_bus::priority::NORMAL
    >;

    can::init();
    channel::init();

    const uint8_t data[] = {
        0x1, 0x2, 0x3
    };

    for (;;) {
        // Send a frame
        can_frame frame;

        for (size_t i = 0; i < 3; i++) {
            frame.data.bytes[i] = data[i];
        }

        frame.length = 3;
        frame.id = 0x03;
        
        channel::send_frame(frame);

        hwlib::cout << "Send frame\r\n";

        // Have we received something?
        while (channel::has_data()) {
            hwlib::cout << "Received data!\r\n";

            const auto bytes = channel::last_frame_data();

            for (const auto byte : bytes) {
                hwlib::cout << byte << ' ';
            }

            hwlib::cout << "\r\n";
        }

        hwlib::wait_ms(500);
    }
}