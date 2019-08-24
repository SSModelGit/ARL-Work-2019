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
 - The script is the `comm_layer.py` script. It instantiates both the `ROSLayer` and `WebClientLayer` classes, and handles the AWS connection process. It retrieves ROS data from the `ROSLayer` class, adds an ID tag to the data, and sends it to AWS via the `WebClientLayer` class. It also retrieves the server response from the `WebClientLayer` class, parses the response, and then sends the appropriate data from AWS into ROS via the `ROSLayer` class.
   - The `comm_layer.py` script also introduces a ROS device ID. This ID is used by the server to identify the device, and so should be unique among all other devices contacting the same server.

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
#### Basic use
The server requires both an IP and a port for initialization - these are specified when initializing the `WebServerData` object. 

For the AWS server, make sure to point the `WebServerClass` to the AWS instance's private IP, and make sure the specified port number is listed within the AWS instance's port connection rules as a TCP port.

#### Interacting with the `ServerData` object
The `ServerData` class interacts with the script via two methods:
 - get_ros_data():
   - This method retrieves all the stored ROS data from all ROS devices. The returned data structure is a Python dictionary. The keys are the ROS device IDs, and the values are the corresponding stored ROS data per device. The ROS data for a given device consists of an array of ROS messages, in the form of dictionaries, i.e. a message of type *geometry_msgs/Twist* would be:
     - {linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}
 - set_server_data(m_id, data):
   - This sets the data to be sent to a ROS device specified by its device ID. The arguments to the method are described below:
     - m_id: This is the unique device ID of the desired ROS device. Choose the appropriate device ID depending on which device the data should be sent to. 
     - data: This is a nested JSON message. There are two top layer fields: *topic* and *data*. The *topic* field is a string of the ROS topic to which the data should be published to. The *data* field contains a JSON message of the ROS message data, in dictionary form as discussed earlier. An example of this is below:
       - ros_data = JSON( { topic: "/topic_name", data: JSON( { data: 1 }) } ), or
       - ros_data = json.dumps( {'topic': "/turtle1/cmd_vel", 'data': json.dumps( {'linear': {'x': 1, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': 0}} ) } )
         - It is important to nest the JSON messages, as after the client loads the JSON message into a dictionary, it will expect to find a JSON messsage under the *data* field.
   
### Running the client
#### Initialization
The first step is to enable websocket connectivity with ROS. To do this, run the `rosbridge_websocket` launch file, using the following command:
```
roslaunch rosbridge_server rosbridge_websocket.launch
```
Note that this makes ROS accessible via a websocket on the device's port numbered 9090. If a different port is preferred, edit the according value in the launch file (*rosbridge_websocket.launch*).

The rest of the initialization steps are adjusting the parameters within the `comm_layer.py` script:
 - `ROSLayer`: Initializing this object requires three inputs - the IP and port of the websocket connected to ROS, and the queue size of the stored subscribed ROS data.
 - `AWSLayer`: Initializing this object requires two inputs - the IP and port of the server. For a server on an AWS instance, use the instance's public IP.
 - ROS device ID: This is the unique ID used by the server to differentiate the ROS device. It can be either a string or an integer.

#### Interacting with the stored data from ROS and AWS
##### Stored ROS data
 - run(topics):
   - This begins the connection to ROS - it requires a list of topics.
   - topics: This is a dictionary of the ROS topics the `ROSLayer` object subscribes to. The keys are strings of the topic names, and the values are strings of the message types, i.e. {"/cmd_vel": "geometry_msgs/Twist"}.
 - get_data_from_buffer(topic):
   - This returns the stored data from the buffer. It will be a list of the latest ROS messages from that topic - the length of the list is specified by the queue size specified during the `ROSLayer` instantiation. The ROS messages will be in dictionary format. 
 - send_data_to_buffer(topic, data):
   - Sends data to a specific topic.
   - The topic field is expected to be a string, ex. "/chatter"
   - The data is expected as a JSON string containing a ROS message, ex. '{"linear": {"y": 0, "x": 1, "z": 0}, "angular": {"y": 0, "x": 0, "z": 1}}'
 - close():
   - Closes the connection to ROS
##### Stored AWS data
 - connect_aws(message):
   - This sends a message to the IP and port specified during the initialization. The message should be a JSON string. Preferably, the message will follow the format expected by the server - a nested JSON string with two top layer fields, 'data' and 'm_id', with the 'data' field being another JSON string of ROS messages.
