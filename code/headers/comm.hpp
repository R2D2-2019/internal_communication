#pragma once

#include <array>
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
        template<can_bus::priority Priority>
        using channel = can_bus::channel_c<bus, Priority>;

        /**
         * Seek all data channels for data to process.
         * Highest priority channels are considered first.
         * If there is no data to process, the resulting array
         * will be empty.
         *
         * @internal
         * @return
         */
        std::array<uint8_t, 8> seek_channels_for_data() const {
            std::array<uint8_t, 8> bytes = {};

            if (channel<can_bus::priority::HIGH>::has_data()) {
                bytes = channel<can_bus::priority::HIGH>::last_frame_data();
            } else if (channel<can_bus::priority::NORMAL>::has_data()) {
                bytes = channel<can_bus::priority::NORMAL>::last_frame_data();
            } else if (channel<can_bus::priority::LOW>::has_data()) {
                bytes = channel<can_bus::priority::LOW>::last_frame_data();
            } else if (channel<can_bus::priority::DATA_STREAM>::has_data()) {
                bytes = channel<can_bus::priority::DATA_STREAM>::last_frame_data();
            }

            return bytes;
        }

        /**
         * A list of packets that this module
         * will listen for on the network.
         */
        std::array<packet_id, 16> listen_for;

    public:
        /**
         * Initialize the communication link for a module.
         * Will initialize the CAN subsystem if not yet initialized.
         */
        explicit comm_c(const std::array<packet_id, 16> &listen_for)
            : listen_for(listen_for) {
            static bool initialized = false;

            if (!initialized) {
                can_bus::comm_module_register_s::clear_register();
                can_bus::controller_c<bus>::init();

                channel<can_bus::priority::HIGH>::init();
                channel<can_bus::priority::NORMAL>::init();
                channel<can_bus::priority::LOW>::init();
                channel<can_bus::priority::DATA_STREAM>::init();

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
                is_suitable_packet_v<T> && !is_extended_packet_v<T>
            >
        >
        void send(const T &data, const can_bus::priority prio = can_bus::priority::NORMAL) const {
            can_bus::detail::_can_frame_s frame{};

            const auto *ptr = reinterpret_cast<const uint8_t *>(&data);

            for (size_t i = 0; i < sizeof(T); i++) {
                frame.data.bytes[i] = ptr[i];
            }

            frame.length = sizeof(T);
            frame.packet_type = packet_type_v<T>;

            if (prio == can_bus::priority::NORMAL) {
                channel<can_bus::priority::NORMAL>::send_frame(frame);
            } else if (prio == can_bus::priority::HIGH) {
                channel<can_bus::priority::HIGH>::send_frame(frame);
            } else if (prio == can_bus::priority::LOW) {
                channel<can_bus::priority::LOW>::send_frame(frame);
            } else {
                channel<can_bus::priority::DATA_STREAM>::send_frame(frame);
            }
        }

        /**
         * Is there any data ready for processing?
         *
         * @return
         */
        bool has_data() const {
            return !rx_buffer.empty();
        }

        /**
        * Get data that is awaiting processing.
        *
        * @tparam T
        * @return
        */
        frame_s get_data() {
            return rx_buffer.copy_and_pop();
        }

        /**
         * Does this module accept the given packet
         * type?
         *
         * @param p
         * @return
         */
        bool accepts_packet_type(const packet_type &p) const override {
            for (const auto &packet : listen_for) {
                if (packet == p) {
                    return true;
                }
            }

            return false;
        }
    };
}