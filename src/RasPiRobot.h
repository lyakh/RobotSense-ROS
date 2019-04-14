/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2019, Guennadi Liakhovetski
 */

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

namespace robot_sense {

class RasPiRobot {
public:
	RasPiRobot(ros::NodeHandle &nh);
	~RasPiRobot() {}
private:
	int GetRpiRevision(void);
	static void driveCallback(const std_msgs::Float32MultiArray::ConstPtr& drive);

	int OC2_PIN;
	ros::Subscriber sub;
};

}
