from webclientlayer import WebClientLayer as WebLayer
from roslayer import ROSLayer
import json


HOST_ID = 1722

if __name__ == '__main__':
    rosLayer = ROSLayer('localhost', 9090, 5)
    awsLayer = WebLayer('localhost', 9091)

    topics = {"/chatter": "geometry_msgs/Twist", "/chatter_two": "geometry_msgs/Twist"}

    ros_data = dict()

    try:
        rosLayer.run(topics)

        while True:
            ros_data["data"] = rosLayer.get_data_from_buffer("/chatter")
            ros_data["m_id"] = HOST_ID
            awsLayer.connect_aws(json.dumps(ros_data))
            rosLayer.send_data_to_topic("/chatter_two", awsLayer.stored_data)

    except KeyboardInterrupt:
        print("Caught keyboard interrupt, exiting")
        rosLayer.close()

    except Exception as e:
        print("unknown error, ouchie.")
        print("error message:", str(e))
        rosLayer.close()
