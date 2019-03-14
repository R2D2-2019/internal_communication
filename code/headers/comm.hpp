#pragma once

#include <type_traits>

#include "channel.hpp"

namespace r2d2 {
    /**
     * Class that represents the boundary between the
     * CAN subsystem and general modules. This class provides
     * a translation between higher level datatypes and the underlying
     * priorities and raw byte transfers.
     */
    class comm_c {
    protected:
        using bus = can_bus::can0;

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

    public:
        /**
         * Initialize the communication link for a module.
         * Will initialize the CAN subsystem if not yet initialized.
         */
        comm_c() {
            static bool initialized = false;

            if (!initialized) {
                can_bus::controller_c<bus>::init();

                channel<can_bus::priority::HIGH>::init();
                channel<can_bus::priority::NORMAL>::init();
                channel<can_bus::priority::LOW>::init();
                channel<can_bus::priority::DATA_STREAM>::init();

                initialized = true;
            }
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
                std::is_pod_v<T> && sizeof(T) <= 8
            >
        >
        void send(const T &data, const can_bus::priority prio = can_bus::priority::NORMAL) const {
            can_bus::detail::_can_frame_s frame{};

            const auto *ptr = reinterpret_cast<const uint8_t *>(&data);

            for (size_t i = 0; i < sizeof(T); i++) {
                frame.data.bytes[i] = ptr[i];
            }

            frame.length = sizeof(T);

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
            if (channel<can_bus::priority::HIGH>::has_data()) {
                return true;
            }

            if (channel<can_bus::priority::NORMAL>::has_data()) {
                return true;
            }

            if (channel<can_bus::priority::LOW>::has_data()) {
                return true;
            }

            return channel<can_bus::priority::DATA_STREAM>::has_data();
        }

        /**
         * Get data that is awaiting processing.
         * Data will be filled in the given type T.
         *
         * @tparam T
         * @return
         */
        template<
            typename T,
            typename = std::enable_if_t<
                std::is_pod_v<T> && sizeof(T) <= 8
            >
        >
        T get_data() const {
            // Array of uint8_t that contains raw representation
            auto bytes = seek_channels_for_data();

            // Cast the array to a pointer of type T and dereference
            return *(reinterpret_cast<T *>(&bytes));
        }

        /**
         * Get data that is awaiting processing.
         * Data will be filled in the given structure.
         *
         * @tparam T
         * @param fill
         */
        template<
            typename T,
            typename = std::enable_if_t<
                std::is_pod_v<T> && sizeof(T) <= 8
            >
        >
        void get_data(T &fill) const {
            // Array of uint8_t that contains raw representation
            const auto bytes = seek_channels_for_data();

            // Consider the variable to be filled as an uint8_t[]
            // and fill it with the given bytes
            auto *ptr = reinterpret_cast<uint8_t *>(&fill);
            for (size_t i = 0; i < sizeof(T); i++) {
                ptr[i] = bytes[i];
            }
        }
    };
}