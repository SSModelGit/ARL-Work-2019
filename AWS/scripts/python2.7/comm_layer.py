## @author Shashank Swaminathan
# @package WebToROS
# Main communication script connecting ROS and the web server.
#
# This is the only script that needs to be run on the ROS device to connect to the web server.
# It instantiates the ROSLayer class to communicate with ROS.
# It instantiates the WebClientLayer class to communicate with the web server.
# Created by Shashank Swaminathan, June 2019, for work in ARL at TMSI, NUS

from webclientlayer import WebClientLayer as WebLayer
from roslayer import ROSLayer
import json


## ID of the ROS Device
#
# Can be either an integer or a string (has to function as a key in a dict)
HOST_ID = 1722

if __name__ == '__main__':
    rosLayer = ROSLayer('localhost', 9090, 5) # Layer that connects script to ROS
    awsLayer = WebLayer('3.0.102.120', 9091) # Layer that connects script to Web Server

    ## topics to subscribe to in ROS
    topics = {"/radio_silence_dev": "std_msgs/String", "/radio_silence_serv": "std_msgs/String",
              "/turtle1/cmd_vel": "geometry_msgs/Twist", "/turtle1/pose": "turtlesim/Pose"}

    ros_data = dict()

    try:
        rosLayer.run(topics) # Pass the topics into the ROSLayer object, and start the connection

        while True:
            # Retrieve data from desired topic - in this case, /turtle1/pose
            # Store it within a larger dictionary under the label of 'data'
            # This makes it easier to parse on the server side
            ros_data["data"] = rosLayer.get_data_from_buffer("/turtle1/pose")

            # Likewise, add a machine ID tag to the dictionary, using the ROS Device ID
            ros_data["m_id"] = HOST_ID

            # print("ROS_DATA:::", ros_data) # Debugging - see what data is read from ROS
            # Send to AWS the data as a JSON string
            ## (bytes, really, but Python2.7 doesn't distinguish, or so I believe)
            awsLayer.connect_aws(json.dumps(ros_data))

            # Retrieve data from AWS, and load it from JSON to a dictionary
            serv_data = json.loads(awsLayer.stored_data)

            # print("SERV_DATA::", serv_data) # Debugging - see what data is read from the server
            # Parse out the topic information and data, and send to ROS
            rosLayer.send_data_to_topic(serv_data['topic'], serv_data['data'])

    except KeyboardInterrupt: # Catch when Ctrl-C is used to stop the system
        print("Caught keyboard interrupt, exiting")
        rosLayer.close()

    except Exception as e: # Unknown error occured, display message.
        print("unknown error, ouchie.")
        print("error message:", str(e))
        rosLayer.close()
