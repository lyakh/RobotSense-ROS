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

void RasPiRobot::driveCallback(const std_msgs::Float32MultiArray::ConstPtr& drive)
{
	if (drive->layout.dim[0].size != 2) {
		fprintf(stderr, "size = %d\n", drive->layout.dim[0].size);
		return;
	}

	std::vector<float>::const_iterator it = drive->data.begin();

	float left = it[0];
	float right = it[1];

	printf("received %f, %f\n", left, right);

#ifdef USE_WIRINGPI
	int left_dir = left < 0 ? LOW : HIGH;
	int right_dir = right < 0 ? LOW : HIGH;
#else
	int left_dir = left < 0 ? 0 : 1;
	int right_dir = right < 0 ? 0 : 1;
#endif

	left = fabs(left);
	right = fabs(right);

	if (left > 1.)
		left = 1.;

	if (right > 1.)
		right = 1.;

	float voltage_ratio = MOTOR_VOLTAGE / BATTERY_VOLTAGE;

#ifdef USE_WIRINGPI
	pwmWrite(LEFT_PWM_PIN, (int)(255. * left));
	digitalWrite(LEFT_DIR_PIN, !left_dir);
	digitalWrite(LEFT_GO_PIN, left_dir);
	pwmWrite(RIGHT_PWM_PIN, (int)(255. * right));
	digitalWrite(RIGHT_DIR_PIN, !right_dir);
	digitalWrite(RIGHT_GO_PIN, right_dir);
#else
	gpioPWM(LEFT_PWM_PIN, (int)(255. * left * voltage_ratio));
	gpioWrite(LEFT_DIR_PIN, !left_dir);
	gpioWrite(LEFT_GO_PIN, left_dir);
	gpioPWM(RIGHT_PWM_PIN, (int)(255. * right * voltage_ratio));
	gpioWrite(RIGHT_DIR_PIN, !right_dir);
	gpioWrite(RIGHT_GO_PIN, right_dir);
#endif
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

	sub = nh.subscribe("/chassis", 10, &driveCallback);
}

}
