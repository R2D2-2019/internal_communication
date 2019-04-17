#pragma once

namespace r2d2 {
    /**
     * All priority levels that can be assigned
     * to packets.
     */
    enum class priority : uint8_t {
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

    /**
     * Helper to get the array index of a given
     * priority.
     *
     * @internal
     * @tparam P
     */
    template<priority P>
    constexpr auto _priority_index = static_cast<uint8_t>(P);
}