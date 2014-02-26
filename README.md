# ME 210 Final Project 

This is the ME210 project code, split into various libraries to ensure modularity and ease of development

## Here is how the files will be organized:

1. Beacon Sensing
    - frequency recognition
    - beacon recognition
2. Tape Sensing
    - tape sensed for different sensors
3. Movement Controls
    - wheel motor direction and output
4. Button Controls
    - linear actuator controls
5. Coin Controls
    - raising coin catch platform
    - controlling coin release
6. State Machine
    - remembers the states
    - controls state from state transitions
    - tracks what actions to take in each state
7. Execute
    - just calls init and loop.

Other notes
    - They will all be using classes?
    - Pins will be defined in state machine and passed down to the library init functions. 

## How to compile it -

Use Execute.ino in the Arduino IDE



Inspiration and overall architecture: https://github.com/rowanc/ME210/tree/master/BrickPush



