#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService

from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2

agent_base = 0
global_pcl_base = 1000
lidar_base = 1001


def main():
    ros_node_name = rospy.get_param("TCP_NODE_NAME", "TCPServer")
    prefix = rospy.get_param("PREFIX", "/server_endpoint")

    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    name = rospy.get_name()  
    server_name = ""
    #Range 1-100 will be reserved for agents
    for i in range(100):
        queryname = prefix + "agent" + str(i)
        if queryname == name:
            print('# Found name! agent' + str(i))
            id = i
            server_name = "agent_server" + str(id)
            identifier = "agent"
            break

    #Range 1000-1100 will be reserved for sensors
    #Range 1000 will be reserved for global point cloud
    for i in range(1):
        queryname = prefix + "global_pcl" + str(i)
        if queryname == name:
            print('# Found name! global_pcl' + str(i))
            id = i + global_pcl_base
            server_name = "global_pcl_server" + str(id - global_pcl_base)
            identifier = "global_pcl"
            break

    #Range 1001-1002 will be reserved for lidar
    for i in range(2):
        queryname = prefix + "lidar" + str(i)
        if queryname == name:
            print('# Found name! lidar' + str(i))
            id = i + lidar_base
            server_name = "lidar_server" + str(id - lidar_base)
            identifier = "lidar"
            break
    #Range 1002-1005 will be reserved for external cameras

    tcp_server = TcpServer(server_name)

    # Agent
    if identifier == "agent":
        idx = id - agent_base
        # Create ROS communication objects dictionary for routing messages
        my_dict = {
            'lidar_scan'+str(idx): RosPublisher('lidar/scan'+str(idx), PointCloud, tcp_server),
            'lidar_pose'+str(idx): RosPublisher('lidar/pose'+str(idx), PoseStamped, tcp_server)
        }

    # Global PCL
    if identifier == "global_pcl":
        idx = id - global_pcl_base
        # Create ROS communication objects dictionary for routing messages
        my_dict = {
            'global_pcl': RosPublisher('global_pcl', PointCloud, tcp_server),
        }

    # Lidar
    if identifier == "lidar":
        idx = id - lidar_base
        # Create ROS communication objects dictionary for routing messages
        my_dict = {
            'lidar_scan'+str(idx): RosPublisher('lidar/scan'+str(idx), PointCloud, tcp_server),
            'lidar_pose'+str(idx): RosPublisher('lidar/pose'+str(idx), PoseStamped, tcp_server)
        }

    tcp_server.source_destination_dict = my_dict

    
    # formatted_float = "{:.2f}".format(a_float)
    print('# Starting ros_node_server = ' + server_name)
    tcp_server.start()
    rospy.spin()

if __name__ == "__main__":
    main()


