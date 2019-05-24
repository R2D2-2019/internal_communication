#pragma once

/*
 * This set of macros is used to quickly add packet helper structs
 * that help the communication system understand how to use
 * and refer to packets. It also checks whether the size of the frame
 * struct is within the hard limits of the protocol.
 * 
 * All frames should be 248 bytes or less, the frame_external_s 
 * is an exception. It is possible there will be another exception for 
 * the robos instructions later.
 */
#define R2D2_STRINGIFY(x) #x
#define R2D2_TO_STRING(x) R2D2_STRINGIFY(x)

#define R2D2_OPTIMISE_STRING(Type, MemberName) \
    template<> \
    struct supports_string_optimisation<Type> : std::true_type {}; \
    \
    template<> \
    struct string_member_offset<Type> { \
        constexpr static uint16_t offset = offsetof(Type, MemberName); \
    };

#define R2D2_OPTIMISE_ARRAY(Type, LengthName, MemberName) \
    template<> \
    struct supports_array_optimisation<Type> : std::true_type {}; \
    \
    template<> \
    struct array_member_offset<Type> { \
        constexpr static uint8_t array_offset = offsetof(Type, MemberName); \
        constexpr static uint8_t length_offset = offsetof(Type, LengthName); \
\
        using array_type = std::remove_pointer_t< \
            std::decay_t< \
                decltype(Type::MemberName) \
            > \
        >; \
    }; \
    static_assert( \
        std::is_same_v<decltype(Type::LengthName), uint8_t>, \
        "The length variable of a array optimised frame should be a uint8_t." \
    );

/**
 * Poisioning is used to prevent people from
 * using structs that are meant purely for the
 * Python bus
 */ 
#define R2D2_POISON_TYPE(Type) _Pragma(R2D2_TO_STRING(GCC poison Type))

#define R2D2_INTERNAL_FRAME_HELPER(Type, EnumVal, ...) \
    template<> \
    struct frame_type_s<Type> { \
        constexpr static frame_id type = frame_type::EnumVal; \
    }; \
    \
    template<> \
    struct frame_data_s<frame_type::EnumVal> { \
        using type = Type; \
    }; \
    \
    static_assert( \
        std::is_same_v<Type, frame_external_s> \
        || sizeof(Type) <= 248, "The size of a frame type should not exceed 248 bytes!" \
    ); \
    \
    __VA_ARGS__

/**
 * Travis doesn't like #pragma pack(1), this define makes
 * it so packing is only done on an ARM target.
 */ 
#if defined(__arm__) || defined(__thumb__)
#define R2D2_PACK_STRUCT _Pragma("pack(1)")
#else
// Empty define
#define R2D2_PACK_STRUCT
#endif

// Tag that indicates that the frame is meant for Python only
#define R2D2_PYTHON_FRAME
