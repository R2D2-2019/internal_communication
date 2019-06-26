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

        //modules attached to can bus
        CAMERA_VISION,
        EXTERNAL_COMMUNICATION,
        DISPLAY,
        END_EFFECTORS,
        FIRE_EXTINGUISHER,
        GAS_DETECTION,
        HAZARD_DETECTION,
        HEALTH_MONITOR,
        LOCATION_DETECTOR,
        MICROPHONE,
        MOVING_PLATFORM,
        ROBOT_ARM,
        POWER,
        SOUND_PLAYBACK,
        WEBCAM,
        TOF_CAMERA,
        VEIN_DETECTION,
        VOICE_INTERACTION,
        LED_STRIP,
        MANUAL_CONTROL,
        THERMAL_VISION,

        SENSOR_HUMIDITY,
        SENSOR_LOAD,
        SENSOR_FLAME,
        SENSOR_GYROSCOPE,
        SENSOR_DISTANCE,
        SENSOR_TEMPERATURE
    };
}