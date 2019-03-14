#include "hwlib.hpp"

#include <comm.hpp>

struct packet_distance_s {
    uint16_t mm;
};

int main(void) {
  // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
    
    hwlib::wait_ms(10);

    r2d2::comm_c comm;

    packet_distance_s distance;
    distance.mm = 404;

    for (;;) {
        hwlib::cout << "Send frame\r\n";
        comm.send(distance, r2d2::can_bus::priority::HIGH);
        comm.send(distance);

        while (comm.has_data()) {
            const auto distance = comm.get_data<packet_distance_s>();
            hwlib::cout << "Received distance: " << distance.mm << "\r\n";
        }

        hwlib::wait_ms(500);
    }
}