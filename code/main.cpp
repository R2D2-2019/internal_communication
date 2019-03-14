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

    using channel_high = r2d2::can_bus::channel_c<
        bus, r2d2::can_bus::priority::NORMAL
    >;

    using channel_normal = r2d2::can_bus::channel_c<
        bus, r2d2::can_bus::priority::NORMAL
    >;

    can::init();
    channel_high::init();
    channel_normal::init();

    const uint8_t data[] = {
        0x1, 0x2, 0x3
    };

    for (;;) {
        auto high_frame = channel_high::bootstrap_frame();
        auto normal_frame = channel_normal::bootstrap_frame();

        for (size_t i = 0; i < 3; i++) {
            high_frame.data.bytes[i] = data[i];
            normal_frame.data.bytes[i] = data[i];
        }

        high_frame.length = 3;
        normal_frame.length = 3;

        channel_high::send_frame(high_frame);
        channel_normal::send_frame(normal_frame);

        hwlib::cout << "Send frame\r\n";

        // Have we received something?
        while (channel_high::has_data()) {
            hwlib::cout << "Received data on the high channel!\r\n";

            const auto bytes = channel_high::last_frame_data();

            for (const auto byte : bytes) {
                hwlib::cout << byte << ' ';
            }

            hwlib::cout << "\r\n";
        }

         while (channel_normal::has_data()) {
            hwlib::cout << "Received data on the normal channel!\r\n";

            const auto bytes = channel_normal::last_frame_data();

            for (const auto byte : bytes) {
                hwlib::cout << byte << ' ';
            }

            hwlib::cout << "\r\n";
        }

        hwlib::wait_ms(500);
    }
}