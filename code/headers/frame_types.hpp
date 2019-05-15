
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
        DISTANCE,
        DISPLAY_FILLED_RECTANGLE,
        DISPLAY_CHARACTER,
        BATTERY_LEVEL,
        UI_COMMAND,

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
     *
     * @tparam T
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

    /**
     * Distance in milimeter
     * 
     * Distance sensor wiki:
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Measuring-distance
     */
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
     * DEPRECATED! Use the frame_display_8x8_character_cursor_s
     * frame tye.
     * 
     * Struct to set a character on a display. This shows
     * a colored character at given location. The character
     * can be any character from the un-extended
     * ascii table (characters 0-127)
     * 
     * Currently only the 8x8 font from hwlib is supported,
     * When extended frames are realised, maybe 16x16 could be
     * implemented.
     * 
     * 
     * 
     * Display wiki:
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Display
     */
    struct frame_display_8x8_character_s {
        // position of character
        uint8_t x;
        uint8_t y;

        // character
        char character;

        // color of pixels
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    };


    /**
     * The display will require each user to claim a cursor
     * these can be used to store data (like position
     * and color).
     */

    enum display_cursor : uint8_t {
    	// Free for any person to use.
    	OPEN_CURSOR,

    	// Don't touch
    	CURSORS_COUNT
    };

    /**
     * Struct to set a character on a display. This shows
     * a colored character at given location. The character
     * can be any character from the un-extended
     * ascii table (characters 0-127)
     * 
     * Currently only the 8x8 font from hwlib is supported,
     * When extended frames are realised, maybe 16x16 could be
     * implemented.
     *
     * For now an alternative to x/y and color based character
     * drawing. 
     */
    struct frame_display_8x8_character_cursor_s {
    	// Targets which cursor to write to. This should be one
    	// your module claimed.
    	display_cursor cursor_id;

    	// The characters to draw
    	char characters[6];

    	// The size of the characters. "abc" = 3.
    	uint8_t amount_characters;
    };

    
    /**
     * This frame will move the targeted cursor to the 
     * given position. (0,0) is the upper left corner.
     * 
     */
    struct frame_display_set_cursor_position_s {
    	// Targets which cursor to write to. This should be one
    	// your module claimed.
    	display_cursor cursor_id;
    	uint8_t cursor_x;
    	uint8_t cursor_y;
    };

    
    /**
     * This frame will set the targeted cursor color to 
     * given colors.
     * 
     */
    struct frame_display_set_cursor_color_s {
    	// Targets which cursor to write to. This should be one
    	// your module claimed.
    	display_cursor cursor_id;
		uint8_t red;
		uint8_t green;
		uint8_t blue; 	
    };

    /**
     *ONLY USABLE IN PYTHON TO PYTHON COMMUNICATION
     *This is a hack that uses the python frame generator to create a frame with strings instead of chars.
     *This conversion does not work in c++.
     *These frames will be sent to swarm management, they only have to call the command with given parameters and send it to the destined robot.
     *
     *Params:
     *	module is the name of the targeted module, mostly used to prevent nameclash
     *	command is the command that needs to be executed, with parameters
     *	destination is used to tell what robot to send the command to
     *
     * SwarmUI wiki:
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Swarm-UI    
     */
    struct frame_ui_command_s {
        char module;
        char command;
        char destination;
    };

    /**
     * Struct that represents the level of 
     * the battery on the robot. 
     * 
     * Power wiki: 
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Power
     */ 
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

    R2D2_INTERNAL_FRAME_HELPER(frame_button_state_s, BUTTON_STATE)
    R2D2_INTERNAL_FRAME_HELPER(frame_activity_led_state_s, ACTIVITY_LED_STATE)
    R2D2_INTERNAL_FRAME_HELPER(frame_distance_s, DISTANCE)
    R2D2_INTERNAL_FRAME_HELPER(frame_display_filled_rectangle_s, DISPLAY_FILLED_RECTANGLE)
    R2D2_INTERNAL_FRAME_HELPER(frame_display_8x8_character_s, DISPLAY_CHARACTER)
    R2D2_INTERNAL_FRAME_HELPER(frame_battery_level_s, BATTERY_LEVEL)
    R2D2_INTERNAL_FRAME_HELPER(frame_ui_command_s, UI_COMMAND)
}
