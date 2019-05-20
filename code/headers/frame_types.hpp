#pragma once

#include "frames/macros.hpp"
#include "frames/definitions.hpp"
#include "frame_enums.hpp"

namespace r2d2 {
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
        DISPLAY_8x8_CHARACTER,
        DISPLAY_8x8_CURSOR_CHARACTER,
        CURSOR_POSITION,
        CURSOR_COLOR,
        UI_COMMAND,        
        BATTERY_LEVEL,
        MANUAL_CONTROL,
        MOVEMENT_CONTROL,
        COORDINATE_STRUCT,
        PATH_STEP,
        COMMAND_LOG,
        COMMAND_STATUS_UPDATE,
        ROBOT_NAMES,
        SWARM_NAMES,
        TEMPERATURE,

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
     * Struct to set a single character on a display. This shows
     * a colored character at given location. The character
     * can be any character from the un-extended
     * ascii table (characters 0-127)
     *
     * Display wiki:
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Display
     */
    R2D2_PACK_STRUCT
    struct frame_display_8x8_character_s {
        // position of character
        uint8_t x;
        uint8_t y;

        // color of pixels
        uint8_t red;
        uint8_t green;
        uint8_t blue;

        // The characters to draw
        // Last element because of string optimisation
        char characters[243];
    };

    /**
     * Struct to set a character on a display. This shows
     * a colored character at given location. The character
     * can be any character from the un-extended
     * ascii table (characters 0-127)
     *
     * For now an alternative to x/y and color based character
     * drawing.
     */
    R2D2_PACK_STRUCT
    struct frame_display_8x8_character_via_cursor_s {
        // Targets which cursor to write to. This should be one
        // your module claimed. The characters will be drawn
        // from the cursor position as starting location.
        uint8_t cursor_id;

        // The characters to draw
        // Last element because of string optimisation
        char characters[247];
    };

    /**
     * This frame will move the targeted cursor to the
     * given position. (0,0) is the upper left corner.
     *
     */
    R2D2_PACK_STRUCT
    struct frame_cursor_position_s {
        // Targets which cursor to write to. This should be one
        // your module claimed. 
        uint8_t cursor_id;

        // new location for the cursor
        uint8_t cursor_x;
        uint8_t cursor_y;
    };

    /**
     * This frame will set the targeted cursor color to
     * given colors.
     *
     */
    R2D2_PACK_STRUCT
    struct frame_cursor_color_s {
        // Targets which cursor to write to. This should be one
        // your module claimed.
        uint8_t cursor_id;

        // cursor color
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    };
    
    /**
     * This frame contains two temperatures.
     * The temperature the sensor is pointed at and
     * the ambient temperature
     * IMPORTANT:
     * All the values must be devided by 10 in order
     * to get the correct value.
     * This is to prevent floating point values.
     */
    R2D2_PACK_STRUCT
    struct frame_temperature_s {
        // This is the (unique) ID of the sensor
        uint32_t id;
        // Ambient temperature multiplied with 10
        int16_t ambient_temperature;
        // Object temperature multiplied with 10
        // Contains the temperature the sensor is pointed at
        int16_t object_temperature;
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
    R2D2_PYTHON_FRAME
    struct frame_ui_command_s {
        // name of the frame or json command which we want to
        // send for evaluation to SMM
        char command;

        // parameters for the frame from frame_name
        char params;

        // destination is used to tell what robot or swarm to
        // send the command to
        char destination;
    };

    /**
     * Only used in python
     * List of all robot names
     * The names of all connected robots will be in this struct, seperated by spaces
     * These names will be used by swarm ui to indicate a destination
     * An example: "robot1 robot2 robot3"
    */
    R2D2_PACK_STRUCT
    struct frame_robot_names_s {
        char names;
    };

    /**
     * Only used in python
     * List of all swarm names
     * The names of all connected swarms will be in this struct, seperated by spaces
     * These names will be used by swarm ui to indicate a destination
     * An example: "swarm1 swarm2 swarm3"
    */
    R2D2_PACK_STRUCT
    struct frame_swarm_names_s {
        char names;
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
        // Battery voltage.
        // The voltage is multiplied by 1000 in this
        // representation. That means that a value of
        // 12.1V will be 12100. This larger value is 
        // used to alleviate the need for floating point numbers.
        // A scale of x1000 is used, because thas is the maximum
        // precision the sensor can read.
        uint32_t voltage;        
        
        // Battery percentage. Between 0 - 100
        uint8_t percentage;
    };

    /**
     * Struct that represent the state
     * of how the robot SHOULD move according the controller.
     * 
     * Manual_control wiki:
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Manual-Control
     * 
     */
    R2D2_PACK_STRUCT
    struct frame_manual_control_s {
        // A value between -100% & 100%
        int8_t speed;

        // A value between -90 & 90 (degrees)
        int8_t rotation;

        // state of the brake button
        bool brake;
    };

    /**
     * Struct that represent the state
     * of how the robot WILL move.
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
     * Struct that represents a coordinate on the planet.
     * 
     * Location_detector wiki:
     * https://github.com/R2D2-2019/location_detector
     */

    R2D2_PACK_STRUCT
    struct frame_coordinate_s {
        // This variable represents the height relative
        // to the average sea level.
        int16_t altitude;
        
        // This variable represents the thousandths
        // seconds of the longitude coordinate.
        uint16_t long_thousandth_sec;
           
        // This variable represents the thousandths
        // seconds of the latitude coordinate.
        uint16_t lat_thousandth_sec;
            
        // This variable represents the degrees of
        // the latitude coordinate.
        uint8_t lat_deg;
        
        // This variable represents the minutes of
        // the latitude coordinate.
        uint8_t lat_min;
        
        // This variable represents the seconds of
        // the latitude coordinate.
        uint8_t lat_sec;
        
        // This variable represents the degrees of
        // the longitude coordinate.
        uint8_t long_deg;
        
        // This variable represents the minutes of
        // the longitude coordinate.
        uint8_t long_min;
        
        // This variable represents the seconds of
        // the longitude coordinate.
        uint8_t long_sec;
        
        // This variable represents the nothern or
        // southern hemisphere the coordinate is located
        // on. North is true, South is false.
        bool north_south_hemisphere;
        
        // This variable represents the eastern or 
        // western hemisphere the coordinate is located
        // on. East is true, West is false.
        bool east_west_hemisphere;
    };
    
    /*
     * Our A-star algorithm outputs a list of 2D vector so
     * the path_id indentifies which list it's from, the 
     * step id is basically the list index. x and y are the
     * 2D vector's attributes.
     * 
     * Navigation wiki:
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Navigation
     */
    R2D2_PACK_STRUCT
    struct frame_path_step_s {
        // x coordinate (in 2d x/y space)
        uint32_t x;

        // y coordinate (in 2d x/y space)
        uint32_t y;

        // sequence integer that indentifies what step in
        // the path we're at.
        uint16_t step_id;

        // unique indentifier for a path so we don't mix
        // up multiple paths.
        uint8_t path_id;
    };
    
    /*
     * This frame will only be used with the python bus.
     * The frame will be responsible for sending log data from
     * SMM to the swarm analytics module.
     *
     * SMM wiki:
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Swarm-Management
     */
    R2D2_PYTHON_FRAME
    struct frame_command_log_s {
        // The current status of the command [for example received,
        // processed, send etc.] This is specified as an integer 
        // since swarm analytics will provide a table containing the
        // explanation for each status.
        uint16_t status;
        
        // This variable will contain the original recieved command
        // type.
        char original_command;

        // This variable will contain the original command data.
        char original_data;
    };
    
    /*
     * This frame will only be used with the python bus.
     * The frame will be responsible for updating the status of a
     * command.
     *
     * SMM wiki:
     * https://github.com/R2D2-2019/R2D2-2019/wiki/Swarm-Management
     */
    R2D2_PYTHON_FRAME
    struct frame_command_status_update_s {
        // The command id for wich the status needs to be updated.
        uint32_t cmd_id;
        
        // The current status of the command, for example received,
        // processed, send etc. This is specified as an integer 
        // since swarm analytics will provide a table containing the
        // explanation for each status.
        uint16_t status;
    };
    

    R2D2_INTERNAL_FRAME_HELPER(frame_button_state_s, BUTTON_STATE)
    R2D2_INTERNAL_FRAME_HELPER(frame_activity_led_state_s, ACTIVITY_LED_STATE)
    R2D2_INTERNAL_FRAME_HELPER(frame_distance_s, DISTANCE)
    R2D2_INTERNAL_FRAME_HELPER(frame_display_filled_rectangle_s, DISPLAY_FILLED_RECTANGLE)

    R2D2_INTERNAL_FRAME_HELPER(
        frame_display_8x8_character_s,
        DISPLAY_8x8_CHARACTER,
        R2D2_OPTIMISE_STRING(frame_display_8x8_character_s, characters)
    )

    R2D2_INTERNAL_FRAME_HELPER(
        frame_display_8x8_character_via_cursor_s,
        DISPLAY_8x8_CURSOR_CHARACTER,
        R2D2_OPTIMISE_STRING(frame_display_8x8_character_via_cursor_s, characters)
    )

    R2D2_INTERNAL_FRAME_HELPER(frame_cursor_position_s, CURSOR_POSITION)
    R2D2_INTERNAL_FRAME_HELPER(frame_cursor_color_s, CURSOR_COLOR)

    R2D2_INTERNAL_FRAME_HELPER(
        frame_ui_command_s,
        UI_COMMAND,
        R2D2_POISON_TYPE(frame_ui_command_s)
    )

    R2D2_INTERNAL_FRAME_HELPER(frame_robot_names, ROBOT_NAMES)
    R2D2_INTERNAL_FRAME_HELPER(frame_swarm_names, SWARM_NAMES)

    R2D2_INTERNAL_FRAME_HELPER(frame_battery_level_s, BATTERY_LEVEL)
    R2D2_INTERNAL_FRAME_HELPER(frame_manual_control_s, MANUAL_CONTROL)
    R2D2_INTERNAL_FRAME_HELPER(frame_movement_control_s, MOVEMENT_CONTROL)
    R2D2_INTERNAL_FRAME_HELPER(frame_coordinate_s, COORDINATE_STRUCT)
    R2D2_INTERNAL_FRAME_HELPER(frame_path_step_s, PATH_STEP)

    R2D2_INTERNAL_FRAME_HELPER(
        frame_command_log_s,
        COMMAND_LOG,
        R2D2_POISON_TYPE(frame_command_log_s)
    )
    
    R2D2_INTERNAL_FRAME_HELPER(
        frame_command_status_update_s,
        COMMAND_STATUS_UPDATE,
        R2D2_POISON_TYPE(frame_command_status_update_s)
    )
    
    R2D2_INTERNAL_FRAME_HELPER(frame_temperature_s, TEMPERATURE)
}
