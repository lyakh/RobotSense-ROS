/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2019, Guennadi Liakhovetski
 */

#include <unistd.h>
#include <sys/syscall.h>   /* For SYS_xxx definitions */
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <semaphore.h>
#ifdef USE_WIRINGPI
#include <wiringPi.h>
#else
#include <pigpio.h>
#endif

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include "RasPiRobot.h"
#include "sg90.h"

#define PIN_TRIGGER 20
#define PIN_ECHO 21
#define PIN_PWM 19

namespace robot_sense {

static pid_t gettid(void)
{
	return syscall(SYS_gettid);
}

class hc_sr04_range {
public:
	hc_sr04_range();
	~hc_sr04_range();
	void spin();
private:
#ifdef USE_WIRINGPI
	static void isr(void *arg);
#else
	static void isr(int gpio, int level, uint32_t tick, void *arg);
#endif

	ros::NodeHandle nh;
	ros::Publisher range_publisher;

	sg90 *motor;
	RasPiRobot *chassis;

	struct timeval tv_recv;
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	unsigned int seq;
};

hc_sr04_range::hc_sr04_range() :
	seq(0)
{
	struct timespec delay = {
		.tv_sec = 0,
		.tv_nsec = 1000000,	// 1ms
	};
#ifdef USE_WIRINGPI
	int ret = wiringPiSetupGpio();

	pwmSetClock(384); //clock at 50kHz (20us tick)
	pwmSetRange(1000); //range at 1000 ticks (20ms)
	pwmSetMode(PWM_MODE_MS);

	pinMode(PIN_TRIGGER, OUTPUT);
	pinMode(PIN_ECHO, INPUT);
	digitalWrite(PIN_TRIGGER, LOW);
#else
	int ret = gpioInitialise();
	if (ret == PI_INIT_FAILED)
		return;

	gpioSetMode(PIN_TRIGGER, PI_OUTPUT);
	gpioSetMode(PIN_ECHO, PI_INPUT);
	gpioWrite(PIN_TRIGGER, 0);

#endif
	nanosleep(&delay, NULL);

#ifdef USE_WIRINGPI
	ret = wiringPiISR(PIN_ECHO, INT_EDGE_BOTH, isr, this);
#else
	ret = gpioSetISRFuncEx(PIN_ECHO, EITHER_EDGE, 0, isr, this);
#endif
	if (ret)
		return;

	pthread_mutex_init(&mutex, NULL);
	pthread_cond_init(&cond, NULL);

	// Publishers
	range_publisher = nh.advertise<sensor_msgs::Range>("sonar", 5);

	motor = new sg90(nh);
	chassis = new RasPiRobot(nh);
}

hc_sr04_range::~hc_sr04_range()
{
}

void hc_sr04_range::isr(
#ifndef USE_WIRINGPI
	int gpio, int level, uint32_t tick,
#endif
	void *arg)
{
	hc_sr04_range *ctx = (hc_sr04_range *)arg;

	gettimeofday(&ctx->tv_recv, NULL);
	fflush(stdout);
	pthread_mutex_lock(&ctx->mutex);
	pthread_cond_signal(&ctx->cond);
	pthread_mutex_unlock(&ctx->mutex);
}

void hc_sr04_range::spin()
{
	struct timespec delay = {
		.tv_sec = 0,
		.tv_nsec = 20000,	// 1ms
	};
	sensor_msgs::Range range_msg;
	ros::Rate rate(2); // 2 hz

	range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
	range_msg.min_range = 0.02;	// 20mm
	range_msg.max_range = 4.0;	// 4m
	range_msg.field_of_view = 0.13;	// 15°

	while (ros::ok()) {
		struct timeval tv;
		unsigned long t0, t;
		unsigned int i;

		// Fire the sonar
		delay.tv_nsec = 20000;		// 20us
#ifdef USE_WIRINGPI
		digitalWrite(PIN_TRIGGER, HIGH);
#else
		gpioWrite(PIN_TRIGGER, 1);
#endif
		nanosleep(&delay, NULL);
#ifdef USE_WIRINGPI
		digitalWrite(PIN_TRIGGER, LOW);
#else
		gpioWrite(PIN_TRIGGER, 0);
#endif
		gettimeofday(&tv, NULL);

		/* Get two interrupts */
		for (i = 0; i < 2; i++) {
			pthread_mutex_lock(&mutex);
			pthread_cond_wait(&cond, &mutex);
			pthread_mutex_unlock(&mutex);

			t = (tv_recv.tv_sec - tv.tv_sec) * 1000000 + tv_recv.tv_usec - tv.tv_usec;
			if (!i)
				t0 = t;

// FIXME: handle +/-Infinity
		}

		printf("diff %f\n", (t - t0) * 0.17);

		range_msg.header.frame_id = "ultrasound_front";
		range_msg.header.stamp = ros::Time::now();
		range_msg.header.seq = seq++;
		range_msg.range = (t - t0) * 0.17;

		range_publisher.publish(range_msg);

		ros::spinOnce();
		rate.sleep();
	}
}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_sense");
	robot_sense::hc_sr04_range one;
	one.spin();

	return 0;
}
