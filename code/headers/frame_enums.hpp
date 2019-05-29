#pragma once

#include <cstdint>

namespace r2d2 {
    /**
     * This file is for all the enum's used in all the frame_types
     *
     */

    /**
     * This enum is for the gas_id that corresponds to the specific gas type.
     */
    enum class gas_type : uint8_t {
        LPG,
        CO,
        SMOKE
    };

    /**
     * The display will require each user to claim a cursor
     * these can be used to store data (like position
     * and color).
     * 
     * Further references to cursor_id will mean this value.
     */
    enum class claimed_display_cursor : uint8_t {
        // Free for any person to use.
        OPEN_CURSOR,
        ROBOS_DISTANCE_CURSOR,
        ROBOS_TEMPERATURE_CURSOR,
        ROBOS_POWER_CURSOR,

        // Don't touch
        CURSORS_COUNT
    };    

    /**
     * This enum is for the different end effector types
     */
    enum class end_effector_type : uint8_t { 
        CLAW, 
        NONE 
    };
}
