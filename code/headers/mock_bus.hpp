#pragma once

#include <base_comm.hpp>
#include <cstring>

namespace r2d2 {
    class mock_comm_c : public base_comm_c {
    protected:

    public:
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
    };
}