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
         * Send the given data with the given priority
         * on the bus.
         *
         * @tparam T
         * @param data
         * @param prio
         */
        template<
            typename T,
            typename = std::enable_if_t<
                is_suitable_frame_v<T> && !is_extended_frame_v<T>
            >
        >
        void send(const T &data, const priority prio = priority::NORMAL) const {
            if (prio == priority::NORMAL) {
                channel<priority::NORMAL>::send_frame(data);
            } else if (prio == priority::HIGH) {
                channel<priority::HIGH>::send_frame(data);
            } else if (prio == priority::LOW) {
                channel<priority::LOW>::send_frame(data);
            } else {
                channel<priority::DATA_STREAM>::send_frame(data);
            }
        }
    };
}