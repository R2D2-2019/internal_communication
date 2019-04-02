# Communication & modules

## Introduction
### What makes a module?
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
            while (comm.has_data()) {
                auto frame = comm.get_data();

                // Process the frame
            }
        }
    }
}
```

In the constructor, the `listen_for_frames` method of the communication instance should be called. This will instruct the communication system to deliver these message types to your module.

You **need** to specify frames you want to receive. This is also the case for requests. Please look at the examples folder for more usage examples.

## Usage (C++)

### Using the interface
#### Request data from the bus
The following code should be used to request data from a module on the bus:
```cpp
// Frame type enum (e.g. frame_type::BUTTON_STATE)
comm.request(frame_type); 
```

#### Processing a request from the bus
If your module receives requests from the bus, it should process them in the `process` function as follows:
```cpp
while (comm.has_data()) {
    auto frame = comm.get_data();

    // Is this frame a request?
    if (frame.request) {
        // Process request, e.g. read a sensor and send value
    }
}
```

#### Send data on the bus
Use the following code to send data on the bus:
```cpp
// Create an instance of a button state frame 
frame_button_state_s button_state;

// Fill the struct with data
button_state.pressed = button.read(); // button is a pin_in

// Send it off!
comm.send(button_state);
```

#### Accepting all frames
Sometimes, you want to accept all frame types. That can be done as follows in the constructor of your module:
```cpp
comm.listen_for_frames({ frame_type::ALL });
```

#### More usage examples
Usage examples can be found in the "examples" folder. Each subfolder in an example represents a module.
These examples are updated to represent the latest version of the library.

### Frames
Sending, requesting or receiving on the bus is done using frames. A frame is a ordered collection of bytes that can be sent or received. Frames are defined in [frame_types.hpp](https://github.com/R2D2-2019/internal_communication/blob/master/code/headers/frame_types.hpp).
Frames are derived from a struct; the struct describes what data the frame contains. A struct is only valid for use as a frame when it is POD type (e.g. [std::is_pod](https://duckduckgo.com/?q=c%2B%2B+pod+types&t=canonical&ia=web) is true for the struct).
Using non-POD structs is not possible; the code won't compile. 

#### A note on frames larger than 8 bytes
At the moment of writing, sending frames that are larger than 8 bytes is still in development and not yet available.
This will be functional and documented in the future.

#### What does a frame look like in code?
A frame consists of three parts, all in [frame_types.hpp](https://github.com/R2D2-2019/internal_communication/blob/master/code/headers/frame_types.hpp).
The first is the enumeration value that refers to the frame:
```cpp
enum frame_type : frame_id {
    // Don't touch
    NONE = 0,

    // The different frame types
    BUTTON_STATE,
    ACTIVITY_LED_STATE,

    // Don't touch
    COUNT
};
```

The second one is the actual struct definition:
```cpp
struct frame_button_state_s {
    bool pressed;
};

struct frame_activity_led_state_s {
    bool state;
};
```
Please note that the name doesn't reflect whether it is a request, instruction or something else.
The name of the struct should simply refer to it's contents, not what it is used for.

The third one is a macro definition at the bottom of the file. This macro definition connects the enumeration value to the struct type:
```cpp
R2D2_INTERNAL_FRAME_HELPER(frame_button_state_s, BUTTON_STATE)
R2D2_INTERNAL_FRAME_HELPER(frame_activity_led_state_s, ACTIVITY_LED_STATE)
```

#### I want to create frame type for my module, how do I add one?
You should create a Pull Request containing the frame definition you want to add.
It has to follow the structure you can see under the previous heading.
In short, your PR should contain:
 - The enumeration value for your frame
 - The struct containing its definition
 - The macro connection the enumeration value to the struct
 
 The leads will need to approve and merge the PR.
 Once merged, the build system will be updated and everyone can pull in the new frame type.


## CAN
### Why a CAN bus?
CAN was chosen because it has the following properties:
 - Multi-master by default
 - Lossless arbitration
 - Fire-and-forget packet-like interface
 - Dedicated hardware support on the Arduino Due
 - Dedicated pinouts on the Due
 - It's a proven industry standard for use in machines and devices (cars, medical, escalators, etc.)
 - Relatively high throughput (1 mbit/s, .5 mbit/s of pure data)
 
 Additionally, other modules within the project probably will use the SPI and I2C interfaces. 
   
### The protocol
While the communication system doesn't depend directly on a specific protocol, the Controller Area Network (CAN) bus is used for intra-microcontroller communication.
CAN is currently implemented in hardware on the Arduino Due. The protocol is used in a way that is a bit different from general use to fit the R2D2 project better.
This doesn't mean the protocol doesn't work with other controllers; everything is within specifications.
The CAN standard 2.0B (extended identifiers, up to 1 mbit/s) is used, FD (flexible data rate) is not supported.

### A note on transceivers
The Arduino Due doesn't come with a transceiver for the CAN bus; this is an additional board that can be purchased and connected. While the CAN bus will function without the transceiver, an eye should be kept at the voltages that the different components will put on the bus.
CAN is a protocol used in cars, machines and other industrial situations, allowing for a wide variety of voltages and loads. Things like electrical motors can put quite a strain on the bus. 
Directly connecting the Due to these types of components will likely damage or destroy the Due. Simply connecting two Due's (as is done in this module) is not an issue.

### Frame structure
A CAN frame within the project has the following structure:
[![CAN-frame-1-1.png](https://i.postimg.cc/9fMyyFJb/CAN-frame-1-1.png)](https://postimg.cc/18kgQPxV)

Explanation:
 - ID: the priority (channel) of the frame. Lower value = higher priority.
 - R/W: whether the frame is a read (request) of write. Write is the default.
 - Free: bits not yet assigned.
 - Reserved: possibly required to support more than 256 frame types in the future.
 - Frame type: the type of the frame (enumeration value) 
 - Sequence id: the id of this frame in the sequence (e.g. frame id is 27 of 30 frames total)
 - Sequence total: the total amount of frames in the sequence
 - Length (protocol defined): the length of the data segment
 - Data (protocol defined): maximum of 8 bytes
 - CRC (protocol defined): the CRC of the frame
