## @author Shashank Swaminathan
# @package WebToROS
# Chatterbot created to test the ROSLayer class
# Created by Shashank Swaminathan, June 2019, for work in ARL at TMSI, NUS

import roslibpy
import Queue
import json
from roslayer import ROSLayer

## Test function to evaluate the subscriber capability of the ROSLayer class
#
# @param connection ROSLayer object
def test_subscriber(connection):
    # Starts a connection to ROS and subscribes to the topic /chatter
    topics = {"/chatter": "geometry_msgs/Twist"}
    connection.run(topics)
    while True:
        # Echo data read from /chatter
        print("\n----")
        print(repr(connection.get_data_from_buffer("/chatter")))

## Test function to evaluate the publisher capability of the ROSLayer class
#
# @param connection ROSLayer object
def test_publisher(connection):
    # Starts a connection to ROS and subscribes to the topic /chatter
    topics = {"/chatter": "geometry_msgs/Twist"}
    connection.run(topics)
    # Initializes a ROS msg to send to ROS
    i = 0
    mes = dict(linear=dict(x=i % 360, y=0, z=0),
               angular=dict(x=0, y=0, z=i % 90))
    while True:
        # Publish data to /chatter
        mes["linear"]["x"] = i % 360
        mes["angular"]["z"] = i % 90
        connection.send_data_to_topic("/chatter", json.dumps(mes))
        i = i+1

## Use this main method along with either the test_subscriber or test_publisher functions to test the ROSLayer class
if __name__ == '__main__':
    # Specify where the ROS master is
    connection = ROSLayer('localhost', 9090, 5)
    try:
        # Run the desired test function here, providing the ROSLayer object as the input
        test_publisher(connection)
    except KeyboardInterrupt:
        connection.close()
