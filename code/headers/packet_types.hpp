#pragma once

#include <cstdint>

/*
 * This macro is used to quickly add packet helper structs
 * that help the communication system understand how to use
 * and refer to packets.
 */
#define R2D2_INTERNAL_FRAME_HELPER(Type, EnumVal) \
    template<> \
    struct frame_type_s<Type> { \
        constexpr static frame_id type = frame_type::EnumVal; \
    }; \
    \
    template<> \
    struct frame_data_s<frame_type::EnumVal> { \
        using type = Type; \
    };

namespace r2d2 {
    /**
     * The underlying type used for packet
     * identifiers.
     */
    using frame_id = uint8_t;

    /**
     * Central definition that is used to
     * determine whether the type T is suitable as
     * a packet type.
     *
     * @tparam T
     */
    template<typename T>
    constexpr bool is_suitable_frame_v = std::is_pod_v<T>;

    /**
     * Is the given packet type
     * extended (spans multiple network layer packets)?
     *
     * @tparam T
     */
    template<typename T>
    constexpr bool is_extended_frame_v = sizeof(T) > 8;

    /**
     * This enum will contain all packet types
     * in the system.
     */
    enum frame_type : frame_id {
        // Don't touch
        NONE = 0,

        // Frame types
        BUTTON_STATE,
        ACTIVITY_LED_STATE,

        // Don't touch
        ALL,
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
    struct frame_type_s {
        constexpr static frame_id type = frame_type::NONE;
    };

    /**
    * Empty base packet.
    */
    struct empty_frame {};

    /**
     * Packet data type accessor
     * base template.
     */
    template<frame_type>
    struct frame_data_s {
        using type = empty_frame;
    };

    /**
     * Helper accessor to get the
     * datatype for the given packet type.
     */
    template<frame_type P>
    using frame_data_t = typename frame_data_s<P>::type;

    /**
     * Helper accessor to get the
     * packet enum value for the given packet type.
     *
     * @tparam T
     */
    template<typename T>
    constexpr frame_id frame_type_v = frame_type_s<T>::type;



    /** USER STRUCTS */


    /**
     * Packet containing the state of 
     * a button.
     */
    struct frame_button_state_s {
        bool pressed;
    };

    /**
     * Packet containing the state of
     * an activity led.
     */
    struct frame_activity_led_state_s {
        bool state;
    };

    R2D2_INTERNAL_FRAME_HELPER(frame_button_state_s, BUTTON_STATE)
    R2D2_INTERNAL_FRAME_HELPER(frame_activity_led_state_s, ACTIVITY_LED_STATE)
}
