#include "hwlib.hpp"

#include <comm.hpp>

int main() {
    using namespace r2d2;

    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;

    hwlib::wait_ms(10);

    r2d2::comm_c comm;

    comm.listen_for_frames(
        {
            r2d2::frame_type::GET_DISTANCE
        }
    );

    packet_distance_s distance;
    distance.mm = 404;

    for (;;) {
        hwlib::cout << "Send frame\r\n";

        comm.request(frame_type::GET_DISTANCE, priority::HIGH);
        comm.send(distance);

        while (comm.has_data()) {
            auto frame = comm.get_data();

            if (frame.request) {
                hwlib::cout << "Received a request!\r\n";
                continue;
            }

            switch (frame.type) {
                case frame_type::GET_DISTANCE: {
                    const auto received = frame.as_frame_type<frame_type::GET_DISTANCE>();
                    hwlib::cout << "Received distance: " << received.mm << "\r\n";
                }
                    break;

                default:
                    break;
            }
        }

        hwlib::wait_ms(500);
    }
}