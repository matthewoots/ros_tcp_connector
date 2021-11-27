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

from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2

def create_dict(dict_type):
    print('# Creating dict {' + dict_type + '}')
    tcp_server = TcpServer(dict_type)    
    my_dict = {}
    if dict_type == "mavros":
        # Mavros
	# Create ROS communication objects dictionary for routing messages
    	my_dict = {

		#'img0': RosPublisher('right/image_rect_raw', Image, tcp_server),	
		#'img1': RosPublisher('left/image_rect_raw', Image, tcp_server),
		#'cam0_left': RosPublisher('right/camera_info', CameraInfo, tcp_server),
		#'cam1_right': RosPublisher('left/camera_info', CameraInfo, tcp_server),

		'local_pos': RosSubscriber('mavros/local_position/pose', PoseStamped, tcp_server),
		'local_vel': RosSubscriber('mavros/local_position/velocity_local', TwistStamped, tcp_server),
		'mavros_state': RosSubscriber('mavros/state', State, tcp_server),
		'batt': RosSubscriber('mavros/battery', BatteryState, tcp_server),
		'global_pos': RosSubscriber('mavros/global_position/global', NavSatFix, tcp_server)
	    }

    elif dict_type == "pcl":
	# pcl
	# Create ROS communication objects dictionary for routing messages
    	my_dict = {'pcl': RosPublisher('pcl', PointCloud, tcp_server)}

    return my_dict

def main():
    ros_node_name = rospy.get_param("TCP_NODE_NAME", "TCPServer")
    tcp_server = TcpServer(ros_node_name)

    tcp_server.source_destination_dict = create_dict(ros_node_name)

    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    # formatted_float = "{:.2f}".format(a_float)
    print('# Current ros_node_name ' + ros_node_name)
    tcp_server.start()
    rospy.spin()

if __name__ == "__main__":
    main()


