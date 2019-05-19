#pragma once

#include <cstdint>

namespace r2d2::can_bus::detail{
    /**
     * Representation of a CAN frame
     * that is put on the network. The frame
     * format has been adjusted from a "normal" CAN
     * frame in the following ways:
     *  - The channel priority decides the ID
     *  - Extended ID mode is in use, but the extended ID is used
     *    to store the packet_type (1 byte), sequence number (5 bits) and
     *    the sequence total (5 bits).
     *
     * @internal
     */
    struct _can_frame_s {
        uint8_t mode; // Read or write
        uint8_t frame_type;
        uint8_t sequence_uid;
        uint8_t sequence_id;
        uint8_t sequence_total;
        uint8_t length;

        // Data
        union {
            struct {
                uint32_t low;
                uint32_t high;
            };

            uint8_t bytes[8];
        } data;
    };
}
