#pragma once

/*
 * This macro is used to quickly add packet helper structs
 * that help the communication system understand how to use
 * and refer to packets.
 */
#define R2D2_INTERNAL_PACKET_HELPER(Type, EnumVal) \
    template<> \
    struct packet_type_s<Type> { \
        constexpr static packet_id type = packet_type::EnumVal; \
    }; \
    \
    template<> \
    struct packet_data_s<packet_type::EnumVal> { \
        using type = Type; \
    };

namespace r2d2 {
    /**
     * The underlying type used for packet
     * identifiers.
     */
    using packet_id = uint8_t;

    /**
     * Central definition that is used to
     * determine whether the type T is suitable as
     * a packet type.
     *
     * @tparam T
     */
    template<typename T>
    constexpr bool is_suitable_packet_v = std::is_pod_v<T>;

    /**
     * Is the given packet type
     * extended (spans multiple network layer packets)?
     *
     * @tparam T
     */
    template<typename T>
    constexpr bool is_extended_packet_v = sizeof(T) > 8;

    /**
     * This enum will contain all packet types
     * in the system.
     */
    enum packet_type : packet_id {
        NONE = 0,

        GET_DISTANCE,

        COUNT
    };

    /** SYSTEM STRUCTS */

    /**
     * Packet enum type accessor
     * base template.
     *
     * @tparam T
     */
    template<typename T>
    struct packet_type_s {
        constexpr static packet_id type = packet_type::NONE;
    };

    /**
    * Empty base packet.
    */
    struct empty_packet {};

    /**
     * Packet data type accessor
     * base template.
     */
    template<packet_type>
    struct packet_data_s {
        using type = empty_packet;
    };

    /**
     * Helper accessor to get the
     * datatype for the given packet type.
     */
    template<packet_type P>
    using packet_data_t = typename packet_data_s<P>::type;

    /**
     * Helper accessor to get the
     * packet enum value for the given packet type.
     *
     * @tparam T
     */
    template<typename T>
    constexpr packet_id packet_type_v = packet_type_s<T>::type;



    /** USER STRUCTS */

    /**
     * Packet containing the request
     * distance.
     */
    struct packet_distance_s {
        // Distance in millimeters
        uint16_t mm;
    };

    R2D2_INTERNAL_PACKET_HELPER(packet_distance_s, GET_DISTANCE)
}
