#pragma once

#include <base_module.hpp>

namespace r2d2::controller {
    class module_c : public base_module_c {
    public:
        /**
         * @param comm
         */
        module_c(base_comm_c &comm)
            : base_module_c(comm) {

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
        void process() override {
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

                frame_activity_led_state_s led_state;
                led_state.state = pressed;

                comm.send(led_state);
            }
        }
    };
}