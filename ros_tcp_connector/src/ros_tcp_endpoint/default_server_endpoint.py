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

def create_dict(id):
    print('# [Creating dict] ' + str(id))
    servername = "server" + str(id)
    tcp_server = TcpServer(servername)    
    my_dict = {}
    if id > 0:
        # Mavros
        # Create ROS communication objects dictionary for routing messages
        my_dict = {

            #'img0': RosPublisher('right/image_rect_raw', Image, tcp_server),	
            #'img1': RosPublisher('left/image_rect_raw', Image, tcp_server),
            #'cam0_left': RosPublisher('right/camera_info', CameraInfo, tcp_server),
            #'cam1_right': RosPublisher('left/camera_info', CameraInfo, tcp_server),

	    ## Subscribing via Unity Ros Package
            # 'local_pos': RosSubscriber('S'+ str(id-1) + '/mavros/local_position/pose', PoseStamped, tcp_server),
            # 'local_vel': RosSubscriber('S'+ str(id-1) + '/mavros/local_position/velocity_local', TwistStamped, tcp_server),
            # 'mavros_state': RosSubscriber('S'+ str(id-1) + '/mavros/state', State, tcp_server),
            # 'batt': RosSubscriber('S'+ str(id) + '/mavros/battery', BatteryState, tcp_server),
            # 'global_pos': RosSubscriber('S'+ str(id-1) + '/mavros/global_position/global', NavSatFix, tcp_server)
        }

    elif id == 0:
        # pcl
        # Create ROS communication objects dictionary for routing messages
        my_dict = {
            'pcl': RosPublisher('pcl', PointCloud, tcp_server),
            'lidar_scan': RosPublisher('lidar/scan', PointCloud, tcp_server),
            'cmd': RosPublisher('cmd', Byte, tcp_server),
	    'lidar_pose': RosPublisher('lidar/pose', PoseStamped, tcp_server)
        }
    
    return my_dict


def main():
    ros_node_name = rospy.get_param("TCP_NODE_NAME", "TCPServer")
    prefix = rospy.get_param("PREFIX", "/server_endpoint")
    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    name = rospy.get_name()  
    id = 0
    for i in range(10):
        queryname = prefix + str(i)
        if queryname == name:
            print('# [Default name]: ' + str(name))
            id = i
            break

    servername = "server" + str(id)
    tcp_server = TcpServer(servername)

    tcp_server.source_destination_dict = create_dict(id)

    
    # formatted_float = "{:.2f}".format(a_float)
    print('# [Current ros_node_name]: ' + servername)
    tcp_server.start()
    rospy.spin()

if __name__ == "__main__":
    main()


