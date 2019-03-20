## LED - controller - button
In this example, three modules on three different microcontrollers work together.
The following modules are present:
 - A `button` module, which reads the state of a button.
 - A `led` module, which controls an activity LED.
 - A `controller` module, which manages the state of the system.
 
 
### Messages

Message flow is as follows:

![Flow](https://i.postimg.cc/RZz4rsFp/Untitled-Diagram-1.png)

1. The controller requests the state of the button.
2. The button module receives the request and responds with the button state.
3. The controller module receives the button state and instructs the LED module to change state.
4. The LED module receives the instruction and changes the LED state.

The controller repeats this process. 
Because the controller requests button state, it is in control of how much data is on the bus and how fast the LED is updated. 