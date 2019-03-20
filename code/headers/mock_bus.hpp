#pragma once

#include <vector>

#include <base_comm.hpp>
#include <cstring>

namespace r2d2 {
    class mock_comm_c : public base_comm_c {
    protected:
        std::vector<frame_s> send_frames;

        /**
          * Send the given data with the given priority.
          *
          * @internal
          * @param buffer
          * @param prio
          */
        void send_impl(const frame_type &type, const uint8_t data[], const priority prio) override {
            frame_s frame{};
            frame.type = type;

            memcpy(
                (void *) frame.bytes,
                (const void *) &data,
                8
            );

            send_frames.push_back(frame);
        }

    public:
        /**
          * Request the given packet on
          * the bus.
          *
          * @param type
          */
        void request(const frame_type &type, const priority prio = priority::NORMAL) override {
            frame_s frame{};
            frame.type = type;
            frame.request = true;

            send_frames.push_back(frame);
        }

        /**
         * Helper function that creates a frame
         * of the given type.
         *
         * @tparam P
         * @param data
         * @return
         */
        template<frame_type P>
        frame_s create_frame(const frame_data_t<P> &data) {
            frame_s frame{};

            frame.type = P;

            memcpy(
                (void *) &frame.bytes,
                (const void *) &data,
                sizeof(frame_data_t<P>)
            );

            return frame;
        }

        /**
         * Get a vector of all frames that
         * have been send using the mock bus.
         *
         * @return
         */
        std::vector<frame_s> &get_send_frames() {
            return send_frames;
        }
    };
}