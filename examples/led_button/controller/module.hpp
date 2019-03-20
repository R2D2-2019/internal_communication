#pragma once

#include <base_comm.hpp>

namespace r2d2::controller {
    class module_c {
    protected:
        /**
         * The communication module
         * instance.
         */
        base_comm_c &comm;

    public:
        /**
         * @param comm
         */
        module_c(base_comm_c &comm)
            : comm(comm) {

            // Set up listeners
            comm.listen_for_frames(
                {
                    r2d2::frame_type::BUTTON_STATE
                }
            );
        }

        /**
         * Let the module process data.
         */
        void process() {
            // Ask the button module for the button state
            comm.request(frame_type::BUTTON_STATE);

            while (comm.has_data()) {
                auto frame = comm.get_data();

                // This module doesn't handle requests
                if (frame.request) {
                    continue;
                }

                const auto pressed = frame.as_frame_type<
                    frame_type::BUTTON_STATE
                >().pressed;

                packet_activity_led_state_s led_state;
                led_state.state = pressed;

                comm.send(led_state);
            }
        }
    }
}