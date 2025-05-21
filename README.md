# Instructions

## Hardware

Print in 3D following model: https://www.thingiverse.com/thing:1015238

## Software 

There are two different use cases:
one with a Joystick shield, one with a Glove with sensors (inertial measurement unit, flex sensor)

Both use cases are stored in dedicated folders where two Arduino projects sit (one for the robotic arm, and one for the glove or joystick that send data over RF)

For the Glove use case, install following libraries:
* In Arduino IDE, install from Library Manager Menu the MPU6050 library by Electronic Cats
* Install as a .ZIP the library for MRF24J RF Module located in examples/ folder

For the Joystick shield use case, install following libraries:
* In Arduino IDE, install from Library Manager Menu the RF24 library by TMRh20