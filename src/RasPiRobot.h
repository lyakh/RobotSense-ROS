/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2019, Guennadi Liakhovetski
 */

#ifndef RASPIROBOT_H
#define RASPIROBOT_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>

namespace robot_sense {

class RasPiRobot {
public:
	RasPiRobot(ros::NodeHandle &nh);
	~RasPiRobot() {}
	void twistCallback(const geometry_msgs::Twist::ConstPtr& twist);
	void sonarCallback(const sensor_msgs::Range::ConstPtr& range);
private:
	int GetRpiRevision(void);
	void doDrive(float linear, float angular);

	int OC2_PIN;

	enum chassis_state {
		CHASSIS_STOP,
		CHASSIS_FORWARD,
		CHASSIS_BACKWARD,
		CHASSIS_LEFT,
		CHASSIS_RIGHT,
		CHASSIS_RECOVER,
	};
	enum chassis_state state;
	unsigned int recover_count;
};

}

#endif
