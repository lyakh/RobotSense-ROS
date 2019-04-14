/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2019, Guennadi Liakhovetski
 */

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

namespace robot_sense {

class listener {
public:
	listener(int argc, char *argv[]);
	~listener() {};
	int control(struct robot_control *control);

private:
	tf2_ros::Buffer tfBuffer;
	ros::Rate *rate;
	tf2_ros::TransformListener *tfListener;
	ros::Publisher angle_publisher;
	ros::Publisher drive_publisher;
	ros::Subscriber sub;
	ros::NodeHandle nh;
};

}
