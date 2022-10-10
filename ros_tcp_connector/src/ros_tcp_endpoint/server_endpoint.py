#!/usr/bin/env python3
# Matthew Woo 2022

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService

from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image

# 1000 agents in total
# 0. imu
# 1. pose
# 2. gps
# 3. rgb camera
# 4. depth
# 5. lidar

# Used to assign topics
def main():
    ros_node_name = rospy.get_param("TCP_NODE_NAME", "TCPServer")

    # Since the prefix is "/server_endpoint_xxx"
    # /server_endpoint_ = 0 to 16
    # agent = 17 to 21
    # global = 17 to 22

    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    name = rospy.get_name()  
    server_name = ""
    query = name[17:len(name)]
    id = 0
    identifier = ""

    # Range 000-999 will be reserved for agents each agents can take 1 port
    if query[0:5] == "agent":
        if query[6:8] == "00":
            id = int(query[8])
        elif query[6] == "0":
            id = int(query[7:9])
        else:
            id = int(query[6:9])     
        print('Found name! agent_' + str(id))
        server_name = "agent_server_" + str(id)
        identifier = "agent"

    # Range 1000-1100 will be reserved for global sensors
    # Range 1000-1004 will be reserved for global point cloud
    if query[0:6] == "global":
        if query[7:10] == "pcl":
            id = int(query[11])
            print('Found name! global_pcl' + str(id))
            server_name = "global_pcl_server_" + str(id)
            identifier = "global_pcl"
    
    # Range 1005-1024 will be reserved for external cameras

    tcp_server = TcpServer(server_name)

    # Agent
    if identifier == "agent":
        # Create ROS communication objects dictionary for routing messages
        my_dict = {
            'imu_'+str(id): RosPublisher('/unity/agent_'+str(id)+'_imu', Imu, tcp_server),
            'pose_'+str(id): RosPublisher('/unity/agent_'+str(id)+'_pose', PoseStamped, tcp_server),
            'gps_'+str(id): RosPublisher('/unity/agent_'+str(id)+'_gps', NavSatFix, tcp_server),
            'rgb_'+str(id): RosPublisher('/unity/agent_'+str(id)+'_rgb', Image, tcp_server),
            'depth_'+str(id): RosPublisher('/unity/agent_'+str(id)+'_depth', Image, tcp_server),
            'lidar_'+str(id): RosPublisher('/unity/agent_'+str(id)+'_lidar', PointCloud, tcp_server)
        }

    # Global pcl
    if identifier == "global_pcl":
        # Create ROS communication objects dictionary for routing messages
        my_dict = {
            'global_pcl_'+str(id): RosPublisher('/unity/global_pcl_'+str(id), PointCloud, tcp_server),
        }

    tcp_server.source_destination_dict = my_dict

    # formatted_float = "{:.2f}".format(a_float)
    print('Starting ros_node_server [' + server_name + ']')
    tcp_server.start()
    rospy.spin()

if __name__ == "__main__":
    main()


