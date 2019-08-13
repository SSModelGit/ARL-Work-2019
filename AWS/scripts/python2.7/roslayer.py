import roslibpy
import Queue
import json

# Connects to a ROS architecture via the package rosbridge and the corresponding roslibpy Python API
# The class provides output of a list of JSON formatted strings of data per each subscribed topic
# Takes JSON formatted strings of data to send to the ROS architecture
# Created by Shashank Swaminathan, June 2019, for work in ARL at TMSI, NUS
# Uses libraries: roslibpy, rosbridge

HOST_ID = 1722

class ROSLayer:
    # Requires: ROS host IP, port number where rosbridge is running, and maximum size of storage buffers
    def __init__(self, host, port, bufsize):
        self.client = roslibpy.Ros(host=host, port=port)
        self.bufsize = bufsize
        self.in_buffer = dict()

    # Requires: Dictionary list of topics, in the form (key: topic name (with / included at start of string), value: message type)
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

    def callback_constructor(self, topic):
        def callback(message):
            print("Callback for topic::", topic)
            print("Data from callback::", message)
            m_jstring = json.dumps(message)
            if self.in_buffer[topic].full() == True:
                self.in_buffer[topic].get_nowait()
                self.in_buffer[topic].put_nowait(m_jstring)
            else:
                self.in_buffer[topic].put_nowait(m_jstring)
        return callback

    # Returns a list of size `bufsize`
    # List of JSON formatted data from ROS topic
    # # Can use json.loads to change data from string form to dictionary form
    def get_data_from_buffer(self, topic):
        return list(self.in_buffer[topic].queue)

    # Sends JSON formatted string to specified ROS topic
    # Does not check if the format matches the topic's message type - please be careful
    def send_data_to_topic(self, topic, data):
        if self.topics[topic].is_advertised == False:
            self.topics[topic].advertise()
        out_message = roslibpy.Message(json.loads(data))
        self.topics[topic].publish(out_message)

    # Invoke when shutting down, to prevent nasty TCP issues
    def close(self):
        for topic in self.topics:
            self.topics[topic].unadvertise()
            self.topics[topic].unsubscribe()

        self.client.terminate()


def test_subscriber(connection):
    topics = {"/chatter": "geometry_msgs/Twist"}
    connection.run(topics)
    while True:
        print("\n----")
        print(repr(connection.get_data_from_buffer("/chatter")))


def test_publisher(connection):
    topics = {"/chatter": "geometry_msgs/Twist"}
    connection.run(topics)
    i = 0
    mes = dict(linear=dict(x=i % 360, y=0, z=0),
               angular=dict(x=0, y=0, z=i % 90))
    while True:
        mes["linear"]["x"] = i % 360
        mes["angular"]["z"] = i % 90
        connection.send_data_to_topic("/chatter", json.dumps(mes))
        i = i+1


# if __name__ == '__main__':
#     connection = ROSLayer('localhost', 9090, 5)
#     try:
#         test_publisher(connection)
#     except KeyboardInterrupt:
#         connection.close()
