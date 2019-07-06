# Using Docker to run ROS on the BlueROV

First, let's address the reason for using Docker when running ROS on the BlueROV. The standard method of integrating ROS with the BlueROV is to leave the Pixhawk (flight controller) and RPi3 (companion computer on board the BlueROV) untouched, and instead to run ROS on the topside computer. There, ROS will communicate with RPi3 via the tether and UDP sockets using `mavros`, connecting the Pixhawk/RPi3 to the ROS architecture. Look at the [Appendix](#the-standard-ros-setup-for-the-bluerov) for a more detailed look at this setup.

Clearly, this is a setup that works for an ROV approach - to make ROS usable autonomously, it has to be placed on board the RPi. However, the native RPi image is in Raspbian, which does not lend itself nicely to installing ROS. The way around this is to "isolate" ROS within a container running on the RPi - using Docker. Essentially, we create a very lightweight imitation of a topside computer within the RPi, and have it operate like the topside computer - UDP connections via `mavros`, etc. - all while remaining within the RPi itself.

## Using Docker to replace the top-side computer
Docker essentially allows us to create an "image" of a system at any given point in time.

# Appendix
## The standard ROS setup for the BlueROV

The standard setup for ROS on the BlueROV is to have:
 - The Pixhawk running ArduSub as the flight controller
 - The Raspberry Pi running the corresponding companion software
 - `mavros` replaces QGroundControl to send and receive data to and from the RPi
   - The two communicate via MAVLink using UDP sockets, through the tethered connection
   - This is how the RPi would normally communicate with QGroundControl. `mavros` simply sits on the port normally allocated for communication with QGC.
 - View [patrickelectric's diagram](https://github.com/patrickelectric/bluerov_ros_playground/#software-layer-diagram) for a nice graphical layout of the standard setup.
