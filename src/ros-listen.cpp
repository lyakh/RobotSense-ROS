/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2019, Guennadi Liakhovetski
 */

#include <math.h>
#include <errno.h>

#include <vector>

#include <ros/ros.h>

#include <tf2/LinearMath/Transform.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/Range.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <tf2_ros/transform_listener.h>

#include "robot_data.h"
#include "ros-listen.h"

#define PI_STEPS 80
#define FRONT_STEPS 80

namespace robot_sense {

static tf2_ros::Buffer *listener_tf_buffer;

static void listener_callback(const sensor_msgs::Range &msg)
{
	const float pi = 3.14159265359;

	try {
		geometry_msgs::TransformStamped transformStamped;
		transformStamped = listener_tf_buffer->lookupTransform("ultrasound_front",
							       "chassis", ros::Time(0));

		tf2::Transform stamped_transform;
		tf2::fromMsg(transformStamped.transform, stamped_transform);
		tf2::Quaternion q = stamped_transform.getRotation();
		tf2::Matrix3x3 m(q);

		tf2Scalar roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		struct robot_state state;

		state.angle = yaw * 100000000;
		state.distance = msg.range * 1000;

		float x = q.x(), y = q.y(), z = q.z();
		fprintf(stderr, "%s(): %d %u angles %f %f %f, rotation %f %f %f\n", __FUNCTION__,
			state.angle, state.distance, x, y, z,
			roll, pitch, yaw);

		client_send(&state);
	} catch (tf2::TransformException &ex) {
		ROS_WARN("transform failed: %s", ex.what());
	}
}

listener::listener(int argc, char *argv[])
{
//	ros::init(argc, argv, "sonar_listener");

	rate = new ros::Rate(10.0);

	listener_tf_buffer = &tfBuffer;

	tfListener = new tf2_ros::TransformListener(tfBuffer, nh),

	sub = nh.subscribe("sonar", 1, listener_callback);
	angle_publisher = nh.advertise<std_msgs::Float32>("/sonar/angle", 5);
	drive_publisher = nh.advertise<std_msgs::Float32MultiArray>("/chassis", 5);
}

int listener::control(struct robot_control *control)
{
	const float pi = 3.14159265359;
	std_msgs::Float32 angle;
	double turn = control->turn / 100000000.;

	if (!nh.ok())
		return -EREMOTEIO;

	angle.data = (float)control->angle / 100000000.;
	angle_publisher.publish(angle);

	/* -1. <= left, right <= 1. */
	double left, right;

	/* pi / 2 -> move forward, -pi / 2 -> move backward */

	if (!control->advance) {
		left = 0.;
		right = 0.;
	} else if (turn >= 0) {
		if (turn >= pi / 2) {
			/* pi/2 <= turn <= pi: left -> max, right slowed */
			left = 1.;
			right = 1. * cos(turn - pi / 2);
		} else {
			/* 0 <= turn <= pi/2: right -> max, left slowed */
			right = 1.;
			left = 1. * cos(turn - pi / 2);
		}
	} else {
		if (turn < -pi / 2) {
			/* -pi <= turn <= -pi/2: left -> max, right slowed */
			left = -1.;
			right = -1. * cos(turn + pi / 2);
		} else {
			/* -pi/2 <= turn <= 0: right -> max, left slowed */
			right = -1.;
			left = -1. * cos(turn + pi / 2);
		}
	}

	printf("send %f, %f\n", left, right);

	std::vector<double> vec = { left, right };
	std_msgs::Float32MultiArray msg;

	// set up dimensions
	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.layout.dim[0].size = vec.size();
	msg.layout.dim[0].stride = 1;
	msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec

	// copy in the data
	msg.data.clear();
	msg.data.insert(msg.data.end(), vec.begin(), vec.end());
	drive_publisher.publish(msg);

	rate->sleep();
	ros::spinOnce();

	return 0;
}

}
