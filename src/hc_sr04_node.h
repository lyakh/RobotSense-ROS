#ifndef HC_SR04_NODE_H
#define HC_SR04_NODE_H

#include <pthread.h>
#include <sys/time.h>

#include "RasPiRobot.h"
#include "sg90.h"

namespace robot_sense {

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

}

#endif
