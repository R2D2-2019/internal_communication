#include <comm.hpp>
#include "hwlib.hpp"


int main() {
    using namespace r2d2;

    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;

    hwlib::wait_ms(1000);

    r2d2::comm_c comm1;
    r2d2::comm_c comm2;

    comm1.listen_for_frames({frame_type::ALL});
    comm2.listen_for_frames({frame_type::DISPLAY_FILLED_RECTANGLE});

    frame_display_filled_rectangle_s state;
    state.x = 0xAAAA;
    state.y = 0xBABA;
    state.width = 1000;
    state.height = 2000;
    state.red = 0xFF;
    state.green = 0xEE;
    state.blue = 0xCC;

    for (;;) {
        state.x++;
        comm1.send(state);
        hwlib::cout << "Com1 Sending: " << hwlib::hex << state.x << hwlib::dec << '\n';

        hwlib::wait_ms(500);

        while(comm2.has_data()){
            auto t = comm2.get_data();

            if(t.request){
                hwlib::cout << "\tCom2 got a request\n";
                continue;
            }

            const auto data = t.as_frame_type<
                frame_type::DISPLAY_FILLED_RECTANGLE>();

            hwlib::cout << "\tCom2 Got frame: " << hwlib::hex << int(data.x) << hwlib::dec << '\n';
        }

        while(comm1.has_data()){
            auto t = comm1.get_data();

            if(t.request){
                hwlib::cout << "\tCom1 got a request\n";
                continue;
            }

            const auto data = t.as_frame_type<
                frame_type::DISPLAY_FILLED_RECTANGLE>();

            hwlib::cout << "\tCom1 Got frame: " << hwlib::hex << int(data.x) << hwlib::dec << '\n';
        }
    }
}