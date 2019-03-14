#pragma once

#include <type_traits>

#include "channel.hpp"

namespace r2d2 {
    class comm_c {
    protected:
        using bus = can_bus::can0;

    public:
        comm_c() {
            static bool initialized = false;

            if (!initialized) {
                can_bus::controller<bus>::init();
                can_bus::channel_c<bus, can_bus::priority::HIGH>::init();
                can_bus::channel_c<bus, can_bus::priority::NORMAL>::init();
                can_bus::channel_c<bus, can_bus::priority::LOW>::init();
                can_bus::channel_c<bus, can_bus::priority::DATA_STREAM>::init();

                initialized = true;
            }
        }

        template<
            typename T,
            typename = std::enable_if_t<
                std::is_pod_v<T> && sizeof(T) <= 8
            >
        >
        void send(const T &data, const can_bus::priority prio = can_bus::priority::NORMAL) {
            can_bus::detail::_can_frame_s frame;

            const auto *ptr = reinterpret_cast<const uint8_t*>(&data);

            for (size_t i = 0; i < sizeof(T); i++) {
                frame.data.bytes[i] = ptr[i];
            }

            frame.length = sizeof(T);            

            if (prio == can_bus::priority::NORMAL) {
                can_bus::channel_c<
                    bus, can_bus::priority::NORMAL
                >::send_frame(frame);
            } else if (prio == can_bus::priority::HIGH) {
                can_bus::channel_c<
                    bus, can_bus::priority::HIGH
                >::send_frame(frame);
            } else if (prio == can_bus::priority::LOW) {
                can_bus::channel_c<
                    bus, can_bus::priority::LOW
                >::send_frame(frame);
            } else {
                can_bus::channel_c<
                    bus, can_bus::priority::DATA_STREAM
                >::send_frame(frame);
            }
        }

        bool has_data() const {
            if (can_bus::channel_c<
                bus, can_bus::priority::HIGH
            >::has_data()) {
                return true;
            }

            if (can_bus::channel_c<
                bus, can_bus::priority::NORMAL
            >::has_data()) {
                return true;
            }
            
            if (can_bus::channel_c<
                bus, can_bus::priority::LOW
            >::has_data()) {
                return true;
            }
            
            if (can_bus::channel_c<
                bus, can_bus::priority::DATA_STREAM
            >::has_data()) {
                return true;
            }

            return false;
        }

        template<
            typename T,
            typename = std::enable_if_t<
                std::is_pod_v<T> && sizeof(T) <= 8
            >
        >
        T get_data() {
            std::array<uint8_t, 8> bytes = {};

            if (can_bus::channel_c<
                bus, can_bus::priority::HIGH
            >::has_data()) {
                bytes =  can_bus::channel_c<
                bus, can_bus::priority::HIGH
            >::last_frame_data(); 
            }

            if (can_bus::channel_c<
                bus, can_bus::priority::NORMAL
            >::has_data()) {
                  bytes = can_bus::channel_c<
                bus, can_bus::priority::NORMAL
            >::last_frame_data(); 
            }
            
            if (can_bus::channel_c<
                bus, can_bus::priority::LOW
            >::has_data()) {
                   bytes = can_bus::channel_c<
                bus, can_bus::priority::LOW
            >::last_frame_data(); 
            }
            
            if (can_bus::channel_c<
                bus, can_bus::priority::DATA_STREAM
            >::has_data()) {
                   bytes =  can_bus::channel_c<
                bus, can_bus::priority::DATA_STREAM
            >::last_frame_data(); 
            }

            T data;
            auto *ptr = reinterpret_cast<uint8_t*>(&data);

            for (size_t i = 0; i < sizeof(T); i++) {
                ptr[i] = bytes[i];
            }

            return data;
        }
    };
}