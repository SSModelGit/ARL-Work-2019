from webclientlayer import WebClientLayer as WebLayer
from roslayer import ROSLayer
import json


# ID of the ROS Device
# Can be either an integer or a string (has to function as a key in a dict)
HOST_ID = 1722

if __name__ == '__main__':
    rosLayer = ROSLayer('localhost', 9090, 5)
    awsLayer = WebLayer('13.229.126.87', 9091)

    topics = {"/radio_silence_dev": "std_msgs/String", "/radio_silence_serv": "std_msgs/String",
              "/turtle1/cmd_vel": "geometry_msgs/Twist", "/turtle1/pose": "turtlesim/Pose"}

    ros_data = dict()

    try:
        rosLayer.run(topics)

        while True:
            ros_data["data"] = rosLayer.get_data_from_buffer("/turtle1/pose")
            ros_data["m_id"] = HOST_ID
            # print("ROS_DATA:::", ros_data)
            awsLayer.connect_aws(json.dumps(ros_data))
            serv_data = json.loads(awsLayer.stored_data)
            # print("SERV_DATA::", serv_data)
            rosLayer.send_data_to_topic(serv_data['topic'], serv_data['data'])

    except KeyboardInterrupt:
        print("Caught keyboard interrupt, exiting")
        rosLayer.close()

    except Exception as e:
        print("unknown error, ouchie.")
        print("error message:", str(e))
        rosLayer.close()
