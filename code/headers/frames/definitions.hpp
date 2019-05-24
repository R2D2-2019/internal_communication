#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

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
     * This struct is specialized to indicate that the
     * type it is specialized for support the string optimzation.
     * 
     * @tparam T
     */ 
    template<typename T>
    struct supports_string_optimisation : std::false_type {};

    /**
     * Struct that stores the offset of
     * the string member that can be optimised against.
     * 
     * @tparam T
     */ 
    template<typename T>
    struct string_member_offset {
        constexpr static uint16_t offset = 0;
    };

    /**
     * Helper accessor to check for string
     * optimisation support on the given type.
     *
     * @tparam T
     */  
    template<typename T>
    constexpr bool supports_string_optimisation_v = supports_string_optimisation<T>::value;

    /**
     * Helper accessor to get the string member
     * offset for the given type.
     * 
     * @tparam T
     */ 
    template<typename T>
    constexpr auto string_member_offset_v = string_member_offset<T>::offset;

    /**
     * This struct is specialized to indicate that the
     * type it is specialized for support the array optimzation.
     * 
     * @tparam T
     */ 
    template<typename T>
    struct supports_array_optimisation : std::false_type {};

    /**
     * Struct that stores the offset of
     * the array members that can be optimised against.
     * 
     * @tparam T
     */ 
    template<typename T>
    struct array_member_offset {
        constexpr static uint8_t array_offset = 0;
        constexpr static uint8_t length = 0;
    };

    /**
     * Helper accessor to check for array
     * optimisation support on the given type.
     *
     * @tparam T
     */  
    template<typename T>
    constexpr bool supports_array_optimisation_v = supports_array_optimisation<T>::value;
}