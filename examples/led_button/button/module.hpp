#pragma once

#include <base_module.hpp>

namespace r2d2::button {
    class module_c : public base_module_c {
    protected:
        /**
         * The button pin.
         */
        hwlib::pin_in &button;

    public:
        /**
         * @param comm
         * @param button
         */
        module_c(base_comm_c &comm, hwlib::pin_in &button)
            : base_module_c(comm), button(button) {

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
            while (comm.has_data()) {
                auto frame = comm.get_data();

                // Only handle requests
                if (!frame.request) {
                    continue;
                }

                // Get button state, create frame and send
                packet_button_state_s button_state;

                button_state.pressed = button.read();

                comm.send(button_state);
            }
        }
    }
}