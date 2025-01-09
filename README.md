### PHX050S-P/Q ROS1 Driver
Using the proprietary SDK provided by Lucid Vision Labs, this ROS1 driver can 
be used to control the PHX050S-P/Q model polarization camera.

It has been tested on Ubuntu 20.04 with ROS Kinetic.

#### CMake and ArenaSDK
The ArenaSDK is provided by Lucid Vision Labs for communicating with their
cameras. Since ArenaSDK is under copyright, it cannot be distributed with this 
program. This driver requires ArenaSDK to be installed. You may also need to 
point CMake at the installation if it cannot find it. Please refer to
`cmake/FindArenaSDK.cmake` based on your installation.

If you are having issues installing ArenaSDK, it is best to try getting one
of the examples included in their distribution working before attempting to 
configure this driver.

#### Topics
The node uses the `image_publisher` package for handling message passing
between nodes. This exposes two topics that other nodes can listen to.

 - `image_raw`
 - `camera_info`

See the `image_publisher` [documentation](http://wiki.ros.org/image_publisher)
for more info.

#### Services
No services are currently available.

