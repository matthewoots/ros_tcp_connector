/*
 * main.cpp
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2022 Matthew (matthewoots at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 * 
 * 
 */


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace std;

class pcl_node
{
private:
    ros::NodeHandle _nh;
    ros::Publisher pcl2_pub;
    ros::Subscriber pcl_sub;

public:

    pcl_node(ros::NodeHandle &nodeHandle)
    {
        pcl2_pub = _nh.advertise<sensor_msgs::PointCloud2>("/pcl2", 10);
        pcl_sub = _nh.subscribe("/pcl", 1, &pcl_node::pcl_callback, this);
        printf("%s[main.cpp] Constructor Setup Ready! \n", KGRN);
    }
    ~pcl_node(){};

    void pcl_callback(const sensor_msgs::PointCloudConstPtr& msg)
    {
        sensor_msgs::PointCloud2 output;
        // Convert to the old point cloud format
        if (!sensor_msgs::convertPointCloudToPointCloud2 (*msg, output))
        {
            ROS_ERROR ("Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
            return;
        }
        ROS_DEBUG ("Publishing a PointCloud2 with %d points.", output.height * output.width);
        pcl2_pub.publish (output);
    }
};



int main(int argc, char **argv)
{
    // ROS init
    ros::init (argc, argv, "pcl2_converter");
    ros::NodeHandle _nh("~"); 

    pcl_node pcl_node(_nh);
    ros::spin ();

    return (0);
}