# Using Docker to run ROS on the BlueROV

## Why use Docker?
The standard method of integrating ROS with the BlueROV is to leave the Pixhawk (flight controller) and RPi3 (companion computer on board the BlueROV) untouched, and instead to run ROS on the topside computer. There, ROS will communicate with RPi3 via the tether and UDP sockets using `mavros`, connecting the Pixhawk/RPi3 to the ROS architecture. Look at the [Appendix](#the-standard-ros-setup-for-the-bluerov) for a more detailed look at this setup.

Clearly, this is a setup that works for an ROV approach - to make ROS usable autonomously, it has to be placed on board the RPi. However, the native RPi image is in Raspbian, which does not lend itself nicely to installing ROS. The way around this is to "isolate" ROS within a container running on the RPi - using Docker.

Essentially, we want to create a very lightweight imitation of a topside computer within the RPi, and have it operate like the topside computer - UDP connections via `mavros`, etc. - all while remaining within the RPi itself. Docker allows us to do just that, with the following properties:
 - It creates an "image" of a system with only the necessary components required to run.
 - As the image is isolated from the host's environment, it can run on various host machines.
   - This allows Docker images to be both lightweight and modular
 - Finally, image aren't directly modified by the user.
   - Docker instantiates images as containers, so whenever the container's edits break, simply instatiate another container to return to the base image.
   - This allows the Docker image to be less prone to accidental failures.

## Using Docker to replace the top-side computer
The key to allowing Docker to replace the top-side computer is to have it emulate the computer's behavior. The RPi only interacts with the top-side computer via one UDP connection - instead, we will have Docker sit in that location. The steps to do so are:
 - Create a Docker image that contains ROS and the necessary ROS packages to connect to the BlueROV.
 - Instantiate a container of this image within the RPi. There are two options to connect the Docker container to the RPi's sockets:
   - Expose the desired port on the Docker container.
   - Set the Docker container to share the host's (RPi's) network.
 - Setup the RPi to start/run this container in the backgroun on startup.

The desired image is built using this [Dockerfile](https://github.com/SSModelGit/DokerFiles/blob/master/bluerov-ros-dock/Dockerfile). It is set to autobuild on Docker Hub on new pushes to GitHub. Pull the image using the following command (use the tag `stable` for confirmed working image builds, use `latest` for the latest version of the image):
```
docker pull sswaminathan235/bluerov-ros-dock:stable
```
Instantiate the container by running the following command:
```
docker run --net=host -v "/dev:/dev:rw" -v "/media:/media:rw" --name=bluerov sswaminathan235/bluerov-ros-dock:stable
```
(OPTIONAL) To allow the container to store results (from mavros, etc.), mount a volume to the container as well, like so:
*TO BE COMPLETED*

## Appendix
### The standard ROS setup for the BlueROV

The standard setup for ROS on the BlueROV is to have:
 - The Pixhawk running ArduSub as the flight controller
 - The Raspberry Pi running the corresponding companion software
 - `mavros` replaces QGroundControl to send and receive data to and from the RPi
   - The two communicate via MAVLink using UDP sockets, through the tethered connection
   - This is how the RPi would normally communicate with QGroundControl. `mavros` simply sits on the port normally allocated for communication with QGC.
 - View [patrickelectric's diagram](https://github.com/patrickelectric/bluerov_ros_playground/#software-layer-diagram) for a nice graphical layout of the standard setup:
 
 <pre>
                      +-----------------------+         +------------------------+
                      |     <b>Raspberry Pi</b>      |         |    <b>Topside Commputer</b>   |
                      |    <b>ip 192.168.2.2</b>     |         |     <b>ip 192.168.2.1</b>     |
                      |                       |         |                        |
+-------+  Telemetry  | +-------------------+ |         |                        |
|Pixhawk<-------------->USB         <b>MAVProxy</b>| |         |                        |
+-------+    Pilot    | +                   + |         | +--------------------+ |
            Control   | |            udpbcast<----------->:14550         <b>MAVROS</b>| |
                      | +-------------------+ |  Pilot  | |(UDP)               | |
                      |                       | Control | |                    | |
                      | +-------------------+ |         | |       (ROS)        | |
+---------+           | CSI+2       <b>raspivid</b>| |         | +------+/mavros+-----+ |
|Raspberry+------------>camera              | |         |           ^            |
| Camera  |           | port                | |         |           |            |
+---------+           | +                   | |         | +---------v----------+ |
                      | |                   | |         | |subs.py      pubs.py| |
                      | +------------+stdout+ |         | |                    | |
                      |                  +    |         | |                    | |
                      |             Raw  |    |         | |                    | |
                      |             H264 |    |         | |                    | |
                      |                  v    |         | |      <b>user.py</b>       | |
                      | +------------+ fdsrc+ |         | |                    | |
                      | |<b>gstreamer</b>          | |         | |                    | |
                      | |                   + |         | :5600 video.py       | |
                      | |             udpsink+----------->(UDP)                | |
                      | +-------------------+ |  Video  | +---------^----------+ |
                      |                       | Stream  |           |            |
                      +-----------------------+         |           +            |
                                                        | +--------/joy--------+ |
                                                        | |<b>joy</b>     (ROS)       | |         +--------+
                                                        | |                  USB<----------+Joystick|
                                                        | +--------------------+ |  Pilot  +--------+
                                                        |                        | Control
                                                        +------------------------+
</pre>

 - Here is a graphical representation of the docker container's behavior within this version of the ROS setup (the graph is derived from the one above):
 ![BlueROV <-> Docker (ROS) Diagram](https://github.com/SSModelGit/ARL-Work-2019/blob/bluerov/BlueROV/docs/docker/BlueROV%20%3C-%3E%20Docker%20%5BROS%5D%20Diagram.png)
