#pragma once

#include <type_traits>

#include "packet_types.hpp"
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
        void send_impl(const frame_type &type, const uint8_t data[], const priority prio) const override {
            if (prio == priority::NORMAL) {
                channel<priority::NORMAL>::send_frame(data);
            } else if (prio == priority::HIGH) {
                channel<priority::HIGH>::send_frame(data);
            } else if (prio == priority::LOW) {
                channel<priority::LOW>::send_frame(data);
            } else {
                channel<priority::DATA_STREAM>::send_frame(data);
            }

            frame_s frame{};
            frame.type = type;

            memcpy(
                (void *) frame.bytes,
                (const void *) &data,
                8
            );

            distribute_frame_internally(frame);
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

            frame_s frame;
            frame.type = type;
            frame.request = true;

            distribute_frame_internally(frame);
        }
    };
}