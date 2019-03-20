#include "ostream"

#include <mock_bus.hpp>

#define CATCH_CONFIG_MAIN
#include <catch.hpp>

using namespace r2d2;

/** RINGBUFFER **/


TEST_CASE("Mock bus accepts packet", "[mock_bus]") {
    mock_comm_c comm;

    comm.listen_for_frames({
        frame_type::GET_DISTANCE
    });

    REQUIRE(comm.accepts_frame(frame_type::GET_DISTANCE));
}

TEST_CASE("Mock bus provides data", "[mock_bus]") {
    mock_comm_c comm;

    comm.listen_for_frames({
        frame_type::GET_DISTANCE
    });

    const auto frame = comm.create_frame<frame_type::GET_DISTANCE>({ 404 });

    comm.accept_frame(frame);

    auto from_comm = comm.get_data();

    REQUIRE(from_comm.type == frame_type::GET_DISTANCE);
    REQUIRE(from_comm.as_frame_type<frame_type::GET_DISTANCE>().mm == 404);
}