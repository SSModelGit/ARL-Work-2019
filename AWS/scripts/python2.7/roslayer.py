## @author Shashank Swaminathan
# @package WebToROS
# Layer connecting ROS to Python
#
# Uses the roslibpy package: https://roslibpy.readthedocs.io/en/latest/
# Requires ROS to run rosbridge_server: http://wiki.ros.org/rosbridge_suite/
# Defines the ROSLayer class, a convenience wrapper for the roslibpy package
# Created by Shashank Swaminathan, June 2019, for work in ARL at TMSI, NUS

import roslibpy
import Queue
import json

HOST_ID = 1722

## Connects to and monitors the state of multiple topics in ROS
#
# Uses JSON to encode ROS messages going into and out of class
# Requires rosbridge_server to be running before starting
class ROSLayer:
    ## Constructor method
    #
    # @param host ROS Master IP
    # @param port Port that rosbridge has connected to
    # @param bufsize Maximum size of queue to store ROS data
    # This does not start the connection to ROS
    def __init__(self, host, port, bufsize):
        self.client = roslibpy.Ros(host=host, port=port)
        self.bufsize = bufsize
        self.in_buffer = dict()

    ## Starts the connection to ROS
    #
    # @param topic_list A dictionary of ROS topics to subscribe to. Keys: String of topic name; Values: String of message type
    def run(self, topic_list):
        self.client.run()

        self.topics = dict()
        for topic in topic_list:
            print(topic)
            self.topics[topic] = roslibpy.Topic(
                self.client, topic, topic_list[topic])
            self.in_buffer[topic] = Queue.Queue(maxsize=self.bufsize)
            spc = self.callback_constructor(topic)
            self.topics[topic].subscribe(spc)

    ## Internal constructor for creating callback functions
    #
    # @param topic Topic to subscribe to
    # @return A function that will handle the callback routine for given topic.
    def callback_constructor(self, topic):
        def callback(message):
            # print("Callback for topic::", topic)
            # print("Data from callback::", message)
            m_jstring = json.dumps(message)
            if self.in_buffer[topic].full() == True:
                self.in_buffer[topic].get_nowait()
                self.in_buffer[topic].put_nowait(m_jstring)
            else:
                self.in_buffer[topic].put_nowait(m_jstring)
        return callback

    ## Class method to retrieve ROS data from class object
    #
    # @param topic String of topic name
    # @return a list of JSON strings containing data from specified ROS topic
    # List is of size `bufsize` (declared in __init__ method)
    def get_data_from_buffer(self, topic):
        return list(self.in_buffer[topic].queue)

    ## Class method to send data to specified ROS topic
    #
    # @param topic String of topic name
    # @param data JSON string of data to be sent to ROS topic. Should formatted to match message type.
    # Note that the method does not check if the data matches the topic's message type. Please be careful.
    def send_data_to_topic(self, topic, data):
        if self.topics[topic].is_advertised == False:
            self.topics[topic].advertise()
        out_message = roslibpy.Message(json.loads(data))
        self.topics[topic].publish(out_message)

    ## Closes connection to ROS cleanly
    def close(self):
        for topic in self.topics:
            self.topics[topic].unadvertise()
            self.topics[topic].unsubscribe()

        self.client.terminate()
