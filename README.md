# Communication module

## What makes a module?
A module has two important moving parts:
 - The communication module that is given to the constructor.
 - The `process` function.

The `process` function **has** to return. It is the responsibility of the owning code to call `process` repeatedly. This allows multiple modules on a single system.

The basic template for a module is as follows:
```cpp
#include <base_module.hpp>

namespace r2d2::module_name {
    class module_c : public base_module_c {
    public:
        /**
         * @param comm
         */
        module_c(base_comm_c &comm) 
            : base_module_c(comm) {
            comm.listen_for_frames(
                // Array of frames
            );
        }

        /**
         * Let the module process data.
         */
        void process() override {
            // Ask the button module for the button state
            while (comm.has_data()) {
                auto frame = comm.get_data();

                // Process the frame
            }
        }
    }
}
```

In the constructor, the `listen_for_frames` method of the communication instance should be called. This will instruct the communication system to deliver these message types to your module.

You **need** to specifiy frames you want to receive. This is also the case for requests. Please look at the examples folder for more usage examples.

## Request data from the bus
The following code should be used to request data from the bus:
```cpp
// Frame type enum (e.g. frame_type::BUTTON_STATE)
comm.request(frame_type); 
```

## Processing a request from the bus
If your module receives requests from the bus, it should process them in the `process` function as follows:
```cpp
while (comm.has_data()) {
    auto frame = comm.get_data();

    // Only handle requests
    if (!frame.request) {
        continue;
    }

    // Process request
}
```

## Send data on the bus
Use the following code to send data on the bus:
```cpp
    // Get button state, create frame and send
    packet_button_state_s button_state;

    button_state.pressed = button.read();

    comm.send(button_state);
```

## Examples
Usage examples can be found in the "examples" folder. Each subfolder in an example represents a module.
These examples are updated to represent the latest version of the library.