#pragma once

#include <cstdint>

namespace r2d2 {
    /**
     * This file is for all the enum's used in all the frame_types
     *
     */

    /**
     * The display will require each user to claim a cursor
     * these can be used to store data (like position
     * and color).
     * 
     * Further references to cursor_id will mean this value.
     */
    enum claimed_display_cursor : uint8_t {
        // Free for any person to use.
        OPEN_CURSOR,

        // Don't touch
        CURSORS_COUNT
    };    
}