
#pragma once

#include <cstdint>
#include <type_traits>

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
#define R2D2_INTERNAL_FRAME_HELPER(Type, EnumVal) \
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
    );

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
        DISTANCE,
        DISPLAY_FILLED_RECTANGLE,
        BATTERY_LEVEL,
        UI_COMMAND,
        MOVEMENT_CONTROL,
        PATH_STEP,

        // Don't touch
        EXTERNAL,
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

    /**
    * A struct that helps to describe
    * an external system address.
    * Might change, depending on the external
    * communication module.
    */
    struct external_id_s {
        // 3th and 4th octet
        uint8_t octets[2];
    };

    /**
     * This frame describes a frame meant for external
     * systems. Tparam T describes the actual frame being send;
     * this structs wraps the internal frame.
     */
    struct frame_external_s {
        uint8_t length;
        external_id_s id;
        frame_type type;

        // NOTE: data should come last; ordering is important
        // for this specific struct!
        uint8_t data[256 - sizeof(uint8_t) - sizeof(external_id_s) - sizeof(frame_type)];
    };

    R2D2_INTERNAL_FRAME_HELPER(frame_external_s, EXTERNAL)

    /** USER STRUCTS */

		
	/** DO NOT REMOVE */
	/** #PythonAnchor# */


    /**
     * Packet containing the state of 
     * a button.
     */
    R2D2_PACK_STRUCT
    struct frame_button_state_s {
        bool pressed;
    };

    /**
     * Packet containing the state of
     * an activity led.
     */
    R2D2_PACK_STRUCT
    struct frame_activity_led_state_s {
        bool state;
    };

    /**
     * Distance in milimeter
     * 
     * Distance sensor wiki:
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Measuring-distance
     */
    R2D2_PACK_STRUCT
    struct frame_distance_s {
        uint16_t mm;
    };
    
    /**
     * Struct to set a rectangle on a display. This fills a 
     * rectangle with the color specified.
     * 
     * Currently we can't fill the bigger screens. When the
     * extended frames are here the position and width/height
     * will change to a uint16_t to support the bigger screens.
     * 
     * Display wiki:
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Display
     */
    R2D2_PACK_STRUCT
    struct frame_display_filled_rectangle_s {
        // position of rectangle
        uint8_t x;
        uint8_t y;

        // dimensions of the rectangle
        uint8_t width;
        uint8_t height;

        // color of pixels
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    };

    /**
     * ONLY USABLE IN PYTHON TO PYTHON COMMUNICATION
     * 
     * This is a hack that uses the python frame generator 
     * to create a frame with strings instead of chars.
     * This conversion does not work in c++. These frames 
     * will be sent to swarm management, they only have to 
     * call the command with given parameters and send it 
     * to the destined robot.
     *
     * SwarmUI wiki:
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Swarm-UI    
     */
    R2D2_PACK_STRUCT
    struct frame_ui_command_s {
        // module is the name of the targeted module, mostly used 
        // to prevent nameclash
        char module;

        // command is the command that needs to be executed, with parameters
        char command;

        // destination is used to tell what robot to send the command to
        char destination;
    };

    /**
     * Struct that represents the level of 
     * the battery on the robot. 
     * 
     * Power wiki: 
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Power
     */ 
    R2D2_PACK_STRUCT
    struct frame_battery_level_s {
        // Battery percentage. Between 0 - 100
        uint8_t percentage;

        // Battery voltage.
        // The voltage is multiplied by 1000 in this
        // representation. That means that a value of
        // 12.1V will be 12100. This larger value is 
        // used to alleviate the need for floating point numbers.
        // A scale of x1000 is used, because thas is the maximum precision
        // the sensor can read.
        uint32_t voltage;
    };

    /**
     * Struct that represent the state
     * of how the robot should move.
     * 
     * Manual_control wiki:
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Manual-Control
     * 
     * Moving Platform wiki:
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Moving-Platform
     */
    R2D2_PACK_STRUCT
    struct frame_movement_control_s {
        // A value between -100% & 100% 
        int8_t speed;

        // A value between -90 & 90 (degrees)
        int8_t rotation;

        // state of the brake button
        bool brake;
    };

    /**
     * Our A-star algorithm outputs a list of 2D vector so the path_id 
     * indentifies which list it's from, the step id is basically 
     * the list index. x and y are the 2D vector's attributes.
     * 
     * Navigation wiki:
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Navigation
     */
    #pragma pack(1)
    struct frame_path_step_s {
        // x coordinate (in 2d x/y space)
        uint32_t x;

        // y coordinate (in 2d x/y space)
        uint32_t y;

        // sequence integer that indentifies what step in the path we're at.
        uint16_t step_id;

        // unique indentifier for a path so we don't mix up multiple paths.
        uint8_t path_id;
    };

    R2D2_INTERNAL_FRAME_HELPER(frame_button_state_s, BUTTON_STATE)
    R2D2_INTERNAL_FRAME_HELPER(frame_activity_led_state_s, ACTIVITY_LED_STATE)
    R2D2_INTERNAL_FRAME_HELPER(frame_distance_s, DISTANCE)
    R2D2_INTERNAL_FRAME_HELPER(frame_display_filled_rectangle_s, DISPLAY_FILLED_RECTANGLE)
    R2D2_INTERNAL_FRAME_HELPER(frame_battery_level_s, BATTERY_LEVEL)
    R2D2_INTERNAL_FRAME_HELPER(frame_ui_command_s, UI_COMMAND)
    R2D2_INTERNAL_FRAME_HELPER(frame_movement_control_s, MOVEMENT_CONTROL)
    R2D2_INTERNAL_FRAME_HELPER(frame_path_step_s, PATH_STEP)

}
