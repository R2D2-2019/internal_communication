#pragma once

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
        packet_type type;
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
                is_suitable_packet_v<T> && !is_extended_packet_v<T>
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
        template<packet_type P>
        auto as_packet_type() -> packet_data_t<P> & {
            return *(reinterpret_cast<packet_data_t<P> *>(&bytes));
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

    public:
        /**
         * Does this module accept the given packet
         * type?
         *
         * @param p
         * @return
         */
        virtual bool accepts_packet_type(const packet_type &p) const = 0;

        /**
         * Accept the given frame. This will
         * insert it into the rx buffer.
         *
         * @param frame
         */
        void accept_packet(const frame_s &frame) {
            rx_buffer.push(frame);
        }
    };
}
