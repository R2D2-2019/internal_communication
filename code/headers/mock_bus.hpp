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
        void send_impl(const frame_type &type, const uint8_t data[], const size_t length, const priority prio) override {
            // TODO: large frame support
            frame_s frame{};
            frame.type = type;
            frame.data = new uint8_t[length];

            for (size_t i = 0; i < length; i++) {
                frame.data[i] = data[i];
            }

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

            const auto length = sizeof(frame_data_t<P>);

            frame.type = P;
            frame.data = new uint8_t[length];

            for (size_t i = 0; i < length; i++) {
                frame.data[i] = reinterpret_cast<const uint8_t*>(&data)[i];
            }            

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
        frame_s get_data() override{
            auto return_frame = send_frames.front();
            send_frames.erase(send_frames.begin());
            return return_frame;
        }
        bool has_data() const override {
            return !send_frames.empty();
        }
    };
}