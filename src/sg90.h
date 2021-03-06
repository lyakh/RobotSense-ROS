/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2019, Guennadi Liakhovetski
 */

#ifndef SG90_H
#define SG90_H

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float32.h>

namespace robot_sense {

class sg90 {
public:
	sg90(ros::NodeHandle &nh);
	~sg90() {}
	void angleCallback(const std_msgs::Float32& msg);
private:
	ros::Publisher angle_publisher;
};

}

#endif
