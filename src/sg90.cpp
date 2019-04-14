/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2019, Guennadi Liakhovetski
 */

#include <ros/ros.h>
#ifdef USE_WIRINGPI
#include <wiringPi.h>
#else
#include <pigpio.h>
#endif
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>

#include "sg90.h"

#define PIN_PWM 19
#ifdef USE_WIRINGPI
#define PI_STEPS 80
#define FRONT_STEPS 80
#else
#define PI_STEPS 1200
#define FRONT_STEPS 1000
#endif

namespace robot_sense {

const float pi = 3.14159265359;

void sg90::angleCallback(const std_msgs::Float32& msg)
{
	int angle = (int)(msg.data * PI_STEPS / pi);

	fprintf(stderr, "%s(): %f %d\n", __FUNCTION__, msg.data, angle);

	if (angle < -PI_STEPS / 4)
		angle = -PI_STEPS / 4;

	if (angle > PI_STEPS / 4)
		angle = PI_STEPS / 4;

#ifdef USE_WIRINGPI
	pwmWrite(PIN_PWM, FRONT_STEPS + angle);
#else
	gpioServo(PIN_PWM, FRONT_STEPS + angle);
#endif

	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "bot";
	transformStamped.child_frame_id = "ultrasound_front";
	transformStamped.transform.translation.x = 0.0;
	transformStamped.transform.translation.y = 0.0;
	transformStamped.transform.translation.z = 0.0;

	tf2::Quaternion q;
	/* 80 units <=> pi clockwise */
	q.setRPY(0, 0, msg.data);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	static tf2_ros::TransformBroadcaster br;
	br.sendTransform(transformStamped);
}

sg90::sg90(ros::NodeHandle &nh)
{
#ifdef USE_WIRINGPI
	pinMode(PIN_PWM, PWM_OUTPUT);
#else
	gpioSetMode(PIN_PWM, PI_OUTPUT);
#endif

	sub = nh.subscribe("/sonar/angle", 10, &angleCallback);
}

}
