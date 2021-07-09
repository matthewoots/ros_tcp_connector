#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)

    # Create ROS communication objects dictionary for routing messages
    tcp_server.source_destination_dict = {
        #'local_pose_enu': RosSubscriber('mavros/local_position/pose', PoseStamped, tcp_server),
        'gndcamera1': RosPublisher('gndcamera1', Image, tcp_server),	
        'gndcamera2': RosPublisher('gndcamera2', Image, tcp_server),
	'aircamera1': RosPublisher('aircamera1', Image, tcp_server)
    }

    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    tcp_server.start()
    rospy.spin()

if __name__ == "__main__":
    main()
