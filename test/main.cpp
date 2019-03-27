#include "ostream"

#include <mock_bus.hpp>

#define CATCH_CONFIG_MAIN

#include <catch.hpp>

using namespace r2d2;

/** RINGBUFFER **/

TEST_CASE("Mock bus accepts packet", "[mock_bus]") {
    mock_comm_c comm;

    comm.listen_for_frames(
        {
            frame_type::BUTTON_STATE
        }
    );

    REQUIRE(comm.accepts_frame(frame_type::BUTTON_STATE));
}

TEST_CASE("Mock bus provides data", "[mock_bus]") {
    mock_comm_c comm;

    comm.listen_for_frames(
        {
            frame_type::BUTTON_STATE
        }
    );

    const auto frame = comm.create_frame<frame_type::BUTTON_STATE>({true});

    comm.accept_frame(frame);

    auto from_comm = comm.get_data();

    REQUIRE(from_comm.type == frame_type::BUTTON_STATE);
    REQUIRE(from_comm.as_frame_type<frame_type::BUTTON_STATE>().pressed);
}

TEST_CASE("Accept all packets works", "[bus]") {
    mock_comm_c comm;

    comm.listen_for_frames({frame_type::ALL});

    REQUIRE(comm.accepts_frame(frame_type::BUTTON_STATE));

    const auto f1 = comm.create_frame<frame_type::BUTTON_STATE>({true});
    const auto f2 = comm.create_frame<frame_type::ACTIVITY_LED_STATE>({false});

    comm.accept_frame(f1);
    comm.accept_frame(f2);

    const auto from1 = comm.get_data();
    const auto from2 = comm.get_data();

    REQUIRE(from1.type == frame_type::ACTIVITY_LED_STATE);
    REQUIRE(from2.type == frame_type::BUTTON_STATE);
}
