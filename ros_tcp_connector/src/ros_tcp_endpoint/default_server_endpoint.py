#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import NavSatFix

from std_msgs.msg import Bool
from std_msgs.msg import Byte

from mavros_msgs.msg import State
from mavros_msgs.msg import ActuatorControl


def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", "TCPServer")
    tcp_server = TcpServer(ros_node_name)

    # Create ROS communication objects dictionary for routing messages
    tcp_server.source_destination_dict = {

        #'Image_Left': RosPublisher('tiscamera_ros/fisheye_left/image_rect_raw', Image, tcp_server),	
        #'Image_Right': RosPublisher('tiscamera_ros/fisheye_right/image_rect_raw', Image, tcp_server),
        #'CameraInfo_Left': RosPublisher('tiscamera_ros/fisheye_left/camera_info', CameraInfo, tcp_server),
        #'CameraInfo_Right': RosPublisher('tiscamera_ros/fisheye_right/camera_info', CameraInfo, tcp_server),
        #'Mapping': RosPublisher('engage_cmd', Bool, tcp_server),
	#'command': RosPublisher('user', Byte, tcp_server)

        'local_pos': RosSubscriber('mavros/local_position/pose', PoseStamped, tcp_server),
        'local_vel': RosSubscriber('mavros/local_position/velocity_local', TwistStamped, tcp_server),
	'mavros_state': RosSubscriber('mavros/state', State, tcp_server),
	'batt': RosSubscriber('mavros/battery', BatteryState, tcp_server),
	'global_pos': RosSubscriber('mavros/global_position/global', NavSatFix, tcp_server)
    }


    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    tcp_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
