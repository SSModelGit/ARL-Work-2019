# AWS Connection scripts
The scripts located within the `scripts` sub-directory are to be used to connect a ROS-running device to AWS (or any other server running websockets). The scripts within the `scripts` folder of this repository are for connecting a ROS device to AWS (or any other accessible websocket server). There are two sets of scripts - one for the device side, and one for the server side. Here is a short description of the two sets of scripts (a more detailed description of the actual code is available within the code docs):

## Server side
The server side consists of a single Python script containing two classes and a `main` method. It is built for Python3.5+.
 - The first class is the auxiliary `ServerData` class. This is a data storage type class: it is meant to store data from the ROS devices, as well as AWS's responses to those devices, within key-value pairs. Each key corresponds to the ID of the ROS device. By using this auxiliary object to store the data, it makes it possible for other threads to access the data within the server.
 - The second class is the main `WebClientServer` class. This initializes and runs a non-blocking TCP/IP websocket server. The server will store data received from ROS within the `ServerData` object, to be later read by other threads. Likewise, it will retrieve data corresponding to the desired device ID from the `ServerData` object to send to the ROS device - this data is set by other threads.
 - The current `main` method implementation is to instatiate a constant message to be sent to all connected ROS devices, and to then start the server's operation. Only one thread is used, hence the constant message. The message is intended for testing within turtlesim, and commands the turtle to move in a circle.

## Client side
*TO BE COMPLETED*

# How to use AWS connection scripts

This usage guide will walk through the steps of preparing and running the scripts on the device and server.

## Server-side connection
### Server setup
There is little work to do on the server side. The scripts have been tested 

*TO BE COMPLETED*
