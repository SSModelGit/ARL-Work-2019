# AWS Connection scripts
The scripts located within the `scripts` sub-directory are to be used to connect a ROS-running device to AWS (or any other server running websockets). The scripts within the `scripts` folder of this repository are for connecting a ROS device to AWS (or any other accessible websocket server). There are two sets of scripts - one for the device side, and one for the server side. Here is a short description of the two sets of scripts (a more detailed description of the actual code is available within the code docs):

## Server side
The server side consists of a single Python script containing two classes and a `main` method. It is written for Python3.5+.
 - The first class is the auxiliary `ServerData` class. This is a data storage type class: it is meant to store data from the ROS devices, as well as AWS's responses to those devices, within key-value pairs. Each key corresponds to the ID of the ROS device. By using this auxiliary object to store the data, it makes it possible for other threads to access the data within the server.
 - The second class is the main `WebClientServer` class. This initializes and runs a non-blocking TCP/IP websocket server. The server will store data received from ROS within the `ServerData` object, to be later read by other threads. Likewise, it will retrieve data corresponding to the desired device ID from the `ServerData` object to send to the ROS device - this data is set by other threads.
 - The current `main` method implementation is to instatiate a constant message to be sent to all connected ROS devices, and to then start the server's operation. Only one thread is used, hence the constant message. The message is intended for testing within turtlesim, and commands the turtle to move in a circle.

## Client side
The client side consists of two Python files defining one class each, and a script that uses both classes to run the client side system. It is written for Python 2.7 and Python3.5+.
 - The first class is the `ROSLayer` class. It utilizes the [`roslibpy`](https://roslibpy.readthedocs.io/en/latest/readme.html) package, and acts as an API between the ROS master and Python. It can subscribe and publish to specified topics. Data received from subscriber callbacks is accessible via class functions.
 - The second class is the `WebClientLayer` class. This class handles the client requests portion of the server-client connection between AWS and the ROS device. The ROS data desired by AWS is sent as the client message in the client request. The class saves the server response.
 - The script is the `comm_layer.py` script. It instantiates both the `ROSLayer` and `WebClientLayer` classes, and handles the AWS connection process. It retrieves ROS data from the `ROSLayer` class and sends it to AWS via the `WebClientLayer` class. It also retrieves the server response from the `WebClientLayer` class, parses the response, and then sends the appropriate data from AWS into ROS via the `ROSLayer` class.

# How to use AWS connection scripts
This usage guide will walk through the steps of preparing and running the scripts on the device and server.

## Setup
### Non-specific setup
Because the connection depends on TCP/IP sockets, ensure that the devices hosting the server and the client have ports exposed for the server and client to access.

### Server setup
In order for the ROS devices to 'see' the server, run the server scripts on a device that is either on the same local network as the ROS devices, or has a public IP. The script is tested on AWS EC2 Ubuntu instances with public IPs.

As the server uses a Python script, it is recommended to use a virtual environment to run the script. The script has been tested in a virtual environment running Python3.5. Beyond that, there are no other steps required of the server during setup.

### Client setup
Because the client connection depends on Python scripts, version 2.7, it is recommended to use a virtual environment running Python2.7.

#### Preparing the `ROSLayer`
In order for the `ROSLayer` class to connect to ROS, both the `roslibpy` package and the `rosbridge_suite` package are required. To install them, follow the steps listed on their documentation.

##### Installing `roslibpy` using pip:
```
pip install roslibpy
```
In case the installation fails, it is recommended to run the following line before re-attempting installation, to handle any potential dependency issues:
```
sudo apt install build-essential libssl-dev libffi-dev python-dev
```
##### Installing `rosbridge_suite`:
Use the apt package manager:
```
sudo apt install ros-$(rosversion -d)-rosbridge-server
```
#### Preparing the `WebClientLayer`
The `WebClientLayer` class uses the `selectors2` package, a version of the `selectors` package ported into Python2.7. Install this package using pip:
```
pip install selectors2
```

## Usage
### Starting & running the server
### Running the client
*TO BE COMPLETED*
