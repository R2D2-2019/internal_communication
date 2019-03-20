/* EXAMPLE TEST CASES */

#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <hwlib.hpp>
#include <mock_bus.hpp>

#include "module.hpp"

using namespace r2d2;

TEST_CASE("LED module outputs the correct LED state", "[led_module]") {
    mock_comm_c mock_bus;

    hwlib::pin_out_store out;

    led::module_c module(mock_bus, out);

    // Create a frame
    auto frame = mock_bus.create_frame<
        frame_type::ACTIVITY_LED_STATE
    >();

    // LED should go to "on"
    frame.as_frame_type<frame_type::ACTIVITY_LED_STATE>().state = true;
    mock_bus.accept_frame(frame);

    // Sanity check
    REQUIRE(out.value == false);

    module.process();

    REQUIRE(out.value == true);
}