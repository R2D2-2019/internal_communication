#pragma once

#define register

#include <type_traits>

#include "frame_types.hpp"
#include "channel.hpp"

namespace r2d2 {
    /**
     * Class that represents the boundary between the
     * CAN subsystem and general modules. This class provides
     * a translation between higher level datatypes and the underlying
     * priorities and raw byte transfers.
     */
    class comm_c : public base_comm_c {
    protected:
        /**
         * Force use of a specific CAN bus
         * for the module.
         */
        using bus = can_bus::can0;

        /**
         * Helper to get a channel type.
         */
        template<priority Priority>
        using channel = can_bus::channel_c<bus, Priority>;

        /**
         * Make sure all other modules
         * on this device get the given frame.
         * 
         * @internal
         * @param frame 
         */
        void distribute_frame_internally(const frame_s &frame) const {
            using regs = can_bus::comm_module_register_s;

            for (uint8_t i = 0; i < regs::count; i++) {
                if (regs::reg[i] != this && regs::reg[i]->accepts_frame(frame.type)) {
                    regs::reg[i]->accept_frame(frame);
                }
            }
        }

        /**
         * Send the given data with the given priority.
         *
         * @internal
         * @param type
         * @param buffer
         * @param prio
         */
        void send_impl(const frame_type &type, const uint8_t data[], const size_t length, const priority prio) override {
            if (prio == priority::NORMAL) {
                channel<priority::NORMAL>::send_frame(type, data, length);
            } else if (prio == priority::HIGH) {
                channel<priority::HIGH>::send_frame(type, data, length);
            } else if (prio == priority::LOW) {
                channel<priority::LOW>::send_frame(type, data, length);
            } else {
                channel<priority::DATA_STREAM>::send_frame(type, data, length);
            }

            // Only internally distribute when needed
            if (can_bus::comm_module_register_s::count <= 1) {
                return;
            }

            frame_s frame{};
            frame.type = type;

            for(uint_fast8_t i = 0; i < 8; i++){
                // frame.bytes[i] = data[i];
            }

            distribute_frame_internally(frame);
        }

        /**
         * Update all the acceptance masks to use the mask
         * 
         * @param mask 
         */
        void update_all_channels(r2d2::frame_id mask){
            // Set the acceptance mask of all the channels to use the mask
            uint32_t normal_mask = channel<priority::NORMAL>::get_mask();
            channel<priority::NORMAL>::set_mask((normal_mask & ~0xFF << 10) | mask << 10);

            uint32_t high_mask = channel<priority::HIGH>::get_mask();
            channel<priority::HIGH>::set_mask((high_mask & ~0xFF << 10) | mask << 10);

            uint32_t low_mask = channel<priority::LOW>::get_mask();
            channel<priority::LOW>::set_mask((low_mask & ~0xFF << 10) | mask << 10);

            uint32_t data_stream_mask = channel<priority::DATA_STREAM>::get_mask();
            channel<priority::DATA_STREAM>::set_mask((data_stream_mask & ~0xFF << 10) | mask << 10);                        
        }

        /**
         * Calculate a acceptance mask depending on the modules on the comm.
         */
        void update_filter() override {
            using regs = can_bus::comm_module_register_s;

            // Start with a full mask.
            r2d2::frame_id mask = ~0x00;

            for (const auto &reg : regs::reg) {
                for (const auto type : reg->get_accepted_frame_types()) {
                    if (type == r2d2::frame_type::ALL) {
                        // Accept all frames and return
                        update_all_channels(0x00);
                        return;
                    }

                    mask &= ~(type);
                }
            }

        	update_all_channels(mask);
        }

    public:
        /**
         * Initialize the communication link for a module.
         * Will initialize the CAN subsystem if not yet initialized.
         *
         * @param listen_for
         */
        comm_c() {
            static bool initialized = false;

            if (!initialized) {
                can_bus::comm_module_register_s::clear_register();
                can_bus::controller_c<bus>::init();

                channel<priority::HIGH>::init();
                channel<priority::NORMAL>::init();
                channel<priority::LOW>::init();
                channel<priority::DATA_STREAM>::init();

                initialized = true;
            }

            can_bus::comm_module_register_s::register_module(this);
        }

        /**
         * Request the given packet on
         * the bus.
         * 
         * @param type 
         */
        void request(const frame_type &type, const priority prio = priority::NORMAL) override {
            if (prio == priority::NORMAL) {
                channel<priority::NORMAL>::request_frame(type);
            } else if (prio == priority::HIGH) {
                channel<priority::HIGH>::request_frame(type);
            } else if (prio == priority::LOW) {
                channel<priority::LOW>::request_frame(type);
            } else {
                channel<priority::DATA_STREAM>::request_frame(type);
            }

            // Only internally distribute when needed
            if (can_bus::comm_module_register_s::count <= 1) {
                return;
            }

            frame_s frame;
            frame.type = type;
            frame.request = true;

            distribute_frame_internally(frame);
        }
    };
}