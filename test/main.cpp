#include <mock_bus.hpp>

#define CATCH_CONFIG_MAIN

#include <catch.hpp>

using namespace r2d2;

TEST_CASE("Mock bus accepts packet", "[mock_bus]") {
    mock_comm_c comm;

    comm.configure(module::TEST, {
        frame_type::BUTTON_STATE
    });

    REQUIRE(comm.accepts_frame(frame_type::BUTTON_STATE));
}

TEST_CASE("Mock bus provides data", "[mock_bus]") {
    mock_comm_c comm;

    comm.configure(module::TEST, {
        frame_type::BUTTON_STATE
    });

    const auto frame = comm.create_frame<frame_type::BUTTON_STATE>({true});

    comm.accept_frame(frame);

    auto from_comm = comm.get_data();

    REQUIRE(from_comm.type == frame_type::BUTTON_STATE);
    REQUIRE(from_comm.as_frame_type<frame_type::BUTTON_STATE>().pressed);
}

TEST_CASE("Accept all packets works", "[bus]") {
    mock_comm_c comm;

    comm.configure(module::TEST, {
        frame_type::ALL
    });

    REQUIRE(comm.accepts_frame(frame_type::BUTTON_STATE));

    const auto f1 = comm.create_frame<frame_type::BUTTON_STATE>({true});
    const auto f2 = comm.create_frame<frame_type::ACTIVITY_LED_STATE>({false});

    comm.accept_frame(f1);
    comm.accept_frame(f2);

    const auto from1 = comm.get_data();
    const auto from2 = comm.get_data();

    REQUIRE(from1.type == frame_type::BUTTON_STATE);
    REQUIRE(from2.type == frame_type::ACTIVITY_LED_STATE);    
}

TEST_CASE("Identity requests get answered implicitly", "[bus]") {
    mock_comm_c basic_module;

    basic_module.configure(module::TEST, {
        frame_type::ALL
    });

    auto req = basic_module.create_frame<frame_type::IDENTITY>();
    req.request = true;
    basic_module.accept_frame(req);

    auto frames = basic_module.get_send_frames();

    REQUIRE(! basic_module.has_data());
    REQUIRE(frames.size() == 1);

    const auto res = frames[0];

    REQUIRE(res.type == frame_type::IDENTITY);
    REQUIRE(res.as_frame_type<frame_type::IDENTITY>().type == module::TEST);  
}
