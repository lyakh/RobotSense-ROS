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
#include <std_msgs/Float32MultiArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>

#include "RasPiRobot.h"

namespace robot_sense {

#define LEFT_GO_PIN 17
#define LEFT_DIR_PIN 4
#define RIGHT_PWM_PIN 14
#define RIGHT_GO_PIN 10
#define RIGHT_DIR_PIN 25
#define LEFT_PWM_PIN 24
#define SW1_PIN 11
#define SW2_PIN 9
#define LED1_PIN 7
#define LED2_PIN 8
#define OC1_PIN 22
#define OC2_PIN_R1 21
#define OC2_PIN_R2 27
#define TRIGGER_PIN 18
#define ECHO_PIN 23

#define BATTERY_VOLTAGE 10.6
#define MOTOR_VOLTAGE 6.

#define CHASSIS_MIN_RANGE 600

void RasPiRobot::doDrive(float linear, float angular)
{
	int left_dir = 0;
	int right_dir = 0;

	float spin = 0.;
	float voltage_ratio = MOTOR_VOLTAGE / BATTERY_VOLTAGE;

	if (angular > 0) {
		right_dir = 1;
		spin = .5;
		if (state != CHASSIS_RECOVER)
			state = CHASSIS_RIGHT;
	} else if (angular < 0) {
		left_dir = 1;
		spin = .5;
		state = CHASSIS_LEFT;
	} else if (linear > 0) {
		left_dir = 1;
		right_dir = 1;
		spin = 1.;
		state = CHASSIS_FORWARD;
	} else if (linear < 0) {
		spin = 1.;
		if (state != CHASSIS_RECOVER)
			state = CHASSIS_BACKWARD;
	} else {
		state = CHASSIS_STOP;
	}

	printf("%s(): left %d right %d spin %f\n", __FUNCTION__, left_dir, right_dir, spin);

	gpioPWM(LEFT_PWM_PIN, (int)(255. * spin * voltage_ratio));
	gpioWrite(LEFT_GO_PIN, !left_dir);
	gpioWrite(LEFT_DIR_PIN, left_dir);
	gpioPWM(RIGHT_PWM_PIN, (int)(255. * spin * voltage_ratio));
	gpioWrite(RIGHT_GO_PIN, !right_dir);
	gpioWrite(RIGHT_DIR_PIN, right_dir);
}

void RasPiRobot::sonarCallback(const sensor_msgs::Range::ConstPtr& range)
{
	switch (state) {
	case CHASSIS_FORWARD:
		if (range->range < CHASSIS_MIN_RANGE) {
			state = CHASSIS_RECOVER;
#ifdef TRY_TO_TURN_ON_TOO_CLOSE
			doDrive(0., 1.);
			recover_count = 3;
#else
			doDrive(-1., 0.);
#endif
		}
		break;
	case CHASSIS_RECOVER:
#ifdef TRY_TO_TURN_ON_TOO_CLOSE
		if (range->range >= CHASSIS_MIN_RANGE && !--recover_count)
			doDrive(1., 0.);
#else
		doDrive(0., 0.);
#endif
	}
}

void RasPiRobot::twistCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
	doDrive(twist->linear.x, twist->angular.z);
}

// Get CPU hardware revision
// 0 unknow
// 1 revision 1
// 2 revision 2
// 3  version B+
int RasPiRobot::GetRpiRevision(void)
{
	char string[128];
	FILE *f = fopen("/proc/cpuinfo", "r");
	int ret, ver;

	if (!f)
		return -errno;

	for (;;) {
		if (!fgets(string, sizeof(string), f))
			break;

		char *rev = strstr(string, "Revision");

		if (!rev)
			continue;

		char *colon = strchr(rev + 8, ':');
		if (!colon)
			continue;

		ret = sscanf(colon + 2, "%x", &ver);
		if (ret < 0)
			return -errno;

		if (!ret)
			return -EINVAL;

		break;
	}

	if (ver > 16)
		return 0;

	if (ver == 16)
		return 3;

	if (ver > 3)
		return 2;

	if (ver > 1)
		return 1;

	return 0;
}

RasPiRobot::RasPiRobot(ros::NodeHandle &nh)
{
	// set OC2 Pin for correct board revision: currently unused
	int ret = GetRpiRevision();
	printf("%s(): revision %d\n", __FUNCTION__, ret);
	if (ret < 0)
		return;

	if (ret == 1)
		OC2_PIN = OC2_PIN_R1;
	else
		OC2_PIN = OC2_PIN_R2;

#ifdef USE_WIRINGPI
	pinMode(LEFT_GO_PIN, PWM_OUTPUT);
	pinMode(LEFT_DIR_PIN, OUTPUT);
	pinMode(RIGHT_GO_PIN, PWM_OUTPUT);
	pinMode(RIGHT_DIR_PIN, OUTPUT);
#else
	gpioSetMode(LEFT_GO_PIN, PI_OUTPUT);
	gpioSetMode(LEFT_DIR_PIN, PI_OUTPUT);
	gpioSetMode(LEFT_PWM_PIN, PI_OUTPUT);
	gpioSetMode(RIGHT_GO_PIN, PI_OUTPUT);
	gpioSetMode(RIGHT_DIR_PIN, PI_OUTPUT);
	gpioSetMode(RIGHT_PWM_PIN, PI_OUTPUT);
#endif
}

}
