#include "hwlib.hpp"

#include <comm.hpp>

struct packet_distance_s {
    uint16_t mm;
};

int main(void) {
  // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
    
    hwlib::wait_ms(10);

    r2d2::comm_c comm({
        r2d2::packet_type::GET_DISTANCE
    });

    packet_distance_s distance;
    distance.mm = 404;

    for (;;) {
        hwlib::cout << "Send frame\r\n";

        comm.send(distance, r2d2::can_bus::priority::HIGH);
        comm.send(distance);

        while (comm.has_data()) {
            auto frame = comm.get_data();

            switch (frame.type) {
                case r2d2::packet_type::GET_DISTANCE: {
                    const auto received = frame.as_packet_type<r2d2::packet_type::GET_DISTANCE>();
                    hwlib::cout << "Received distance: " << received.mm << "\r\n";
                } break;

                default: break;
            }
        }

        hwlib::wait_ms(500);
    }
}