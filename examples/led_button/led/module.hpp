#pragma once

#include <base_comm.hpp>

namespace r2d2::led {
    class module_c {
    protected:
        /**
         * The communication module
         * instance.
         */
        base_comm_c &comm;

        /**
         * The LED pin.
         */
        hwlib::pin_out &led;

    public:
        /**
         * @param comm
         * @param led
         */
        module_c(base_comm_c &comm, hwlib::pin_out &led)
            : comm(comm), led(led) {

            // Set up listeners
            comm.listen_for_frames({
                r2d2::frame_type::ACTIVITY_LED_STATE
            });
        }

        /**
         * Let the module process data.
         */
        void process() {
            while (comm.has_data()) {
                auto frame = comm.get_data();

                // Don't handle requests
                if (frame.request) {
                    continue;
                }

                // Get the data from the frame
                const auto data = frame.as_frame_type<
                    frame_type::ACTIVITY_LED_STATE
                >();

                // Set the LED state to the given value
                led.write(data.state);
            }
        }
    };
}