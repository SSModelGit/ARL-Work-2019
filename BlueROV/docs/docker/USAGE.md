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

### Setup the RPi3B+ for Docker Use

#### Connect to the RPi

Connect to the RPi via an ethernet cable, and use `ifconfig` to place the ethernet connection on the same IP network as the RPi (i.e. if the RPi has an IP of `192.169.2.0`, then use `ifconfig` to place the ethernet connection at `192.168.2.1`). Make sure that the RPi has enabled/allowed SSH connections, and then use command below to SSH into the Raspberry Pi:
```
ssh pi@raspberrypi.local
```
The default password for the RPi is `raspberry`. Currently, the lab's test RPi platform's password is `bluerov-ARL-2019`.

#### Install Docker on the Raspberry Pi

Because the RPi platform uses the `arm` architecture, and not the `x86_64` architecture, we use the Debian armhf/arm64 specific installs. There are a few ways to proceed with the installation, depending on the system at hand:

##### RPi not running Rasbian Buster:

This one line setup script will run through the installation process:
```
curl -sSL get.docker.com | sh
```

##### RPi running Raspbian Buster:

Read through this [issue thread](https://github.com/docker/for-linux/issues/709) for solutions pertaining to the particular issue at hand. The solution that worked for the testing platform in the lab was a manual install of the docker packages, detailed in [this guide](https://docs.docker.com/install/linux/docker-ce/debian/). If the thread does not provide a solution, another alternative is to try installing Docker via `snapd`.

### Install the desired Docker image

#### Setup the Docker Image (*interim*)

Run the command `sudo docker pull arm32v7/ubuntu:16.04` to pull the appropriate base image from Docker Hub.

Within the container, run the standard ROS installation commands.
```
apt-get update && apt install git wget curl
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt-get update && apt-get install ros-kinetic-base
rosdep init && rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> /etc/environment
apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```
Now, install the custom  BlueROV ROS packages via the following commands.
```
apt-get install ros-kinetic-mavros* ros-kinetic-joy
mkdir -p /catkin_ws/src/ && cd /catkin_ws/src/
git clone https://github.com/SSModelGit/bluerov-ros-pkg.git && cd bluerov-ros-pkg && git checkout arl
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
bash install_geographiclib_datasets.sh && rm install_geographiclib_datasets.sh
cd /catkin_ws/ && catkin_make
echo "source /catkin_ws/devel/setup.bash" >> /etc/environment
```

#### Installing the desired Docker image (*does not yet work*)
The desired image is built using this [Dockerfile](https://github.com/SSModelGit/DokerFiles/blob/master/bluerov-ros-dock/Dockerfile). It is set to autobuild on Docker Hub on new pushes to GitHub. Pull the image using the following command (use the tag `stable` for confirmed working image builds, use `latest` for the latest version of the image):
```
docker pull sswaminathan235/bluerov-ros-dock:stable
```

### Readying the Docker container

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
