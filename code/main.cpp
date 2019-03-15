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

    // Possibility 1
    int counter = 0;
    comm.on_receive(r2d2::packet_type::GET_DISTANCE, [&counter](const r2d2::frame_s &frame) {
        hwlib::cout << "Received frame of type: " << static_cast<uint8_t>(frame.type) << "\r\n";
        counter += 1;
    });

    packet_distance_s distance;
    distance.mm = 404;


    // Possibility 2:
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