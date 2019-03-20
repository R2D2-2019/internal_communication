#pragma once

#include <array>
#include <type_traits>

#include "packet_types.hpp"

namespace r2d2 {
    /**
     * All priority levels that can be assigned
     * to packets.
     */
    enum class priority {
        // High priority packet
            HIGH = 0,

        // Normal priority packet, default
            NORMAL = 1,

        // Low priority packet, used when there is no hard time constraint on the delivery of the data
            LOW = 2,

        // Data stream packet. Used when a stream of packets (e.g. video data) is put on the bus.
        // Assigning this will given these packets the lowest priority, preventing the large data stream
        // from clogging up the bus.
            DATA_STREAM = 3
    };

    struct frame_s {
        frame_type type;
        bool request;
        uint8_t bytes[8];

        /**
         * Consider the data in the frame as
         * the given datatype. The returned data is
         * a reference and as such non-owning.
         *
         * @tparam T
         * @return
         */
        template<
            typename T,
            typename = std::enable_if_t<
                is_suitable_frame_v < T> && !is_extended_frame_v <T>
            >
        >
        T &as_type() {
            return *(reinterpret_cast<T *>(&bytes));
        }

        /**
         * Consider the data in the frame
         * as the datatype associated with the given
         * packet type. The returned data is
         * a reference and as such non-owning.
         *
         * @tparam P
         * @return
         */
        template<frame_type P>
        auto as_frame_type() -> frame_data_t <P> & {
            return *(reinterpret_cast<frame_data_t<P> *>(&bytes));
        }
    };

    /**
     * Base class for the communication module.
     * This is used to prevent circular dependencies and
     * allow for mocks.
     */
    class base_comm_c {
    protected:
        ringbuffer_c<frame_s, 32> rx_buffer;

        /**
         * A list of packets that this module
         * will listen for on the network.
         */
        std::array<frame_id, 8> listen_for{};

        /**
         * Send the given data with the given priority.
         *
         * @internal
         * @param buffer
         * @param prio
         */
        virtual void send_impl(const frame_type &type, const uint8_t data[], const priority prio) = 0;

    public:
        /**
         * Request the given packet on
         * the bus.
         * 
         * @param type 
         */
        virtual void request(const frame_type &type, const priority prio = priority::NORMAL) = 0;

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
        void send(const T &data, const priority prio = priority::NORMAL) {
            uint8_t buffer[8] = {};

            memcpy(
                (void *) buffer,
                (const void *) &data,
                sizeof(T)
            );

            send_impl(
                static_cast<frame_type>(frame_type_v<T>), buffer, prio
            );
        }

        /**
         * Listen for the given list of packets.
         *
         * @param listen_for
         */
        void listen_for_frames(const std::array<frame_id, 8> listen_for) {
            this->listen_for = listen_for;
        }

        /**
         * Accept the given frame. This will
         * insert it into the rx buffer.
         *
         * @param frame
         */
        void accept_frame(const frame_s &frame) {
            rx_buffer.push(frame);
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
         * Get the list of packets that this module
         * listens for.
         * @return
         */
        std::array<frame_id, 8> const& get_accepted_frame_types() const {
            return listen_for;
        }

        /**
         * Does this module accept the given packet
         * type?
         *
         * @param p
         * @return
         */
        bool accepts_frame(const frame_type &p) const {
            for (const auto &packet : get_accepted_frame_types()) {
                if (packet == p) {
                    return true;
                }
            }

            return false;
        }
    };
}
