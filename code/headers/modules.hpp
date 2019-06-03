#pragma once

namespace r2d2 {
    /**
     * A list of all modules on the bus.
     * This is required so that the system can 
     * identify which modules are connected on the robot.
     */ 
    enum class module : uint8_t {
        NONE,
        
        // Module types used for tests and examples
        TEST,
        BUTTON,
        CONTROLLER,
        LED,

        POWER
    };
}