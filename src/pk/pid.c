#include "pid.h"

/**
 * Initialize a PID system
 */
void pid_init(PID* tracker, double p, double i, double d){
	tracker->kp = p;
	tracker->ki = i;
	tracker->kd = d;
	tracker->proportion = 0;
	tracker->integral = 0;
	tracker->derivative = 0;
	tracker->current.ticks = 0;
	tracker->current.value = 0;
	tracker->prev.ticks = 0;
	tracker->prev.value = 0;
}

/**
 * Samples a data from feedback.
 * Must be called in a regular time interval.
 */
void pid_sampling(PID* tracker, int reading){
	tracker->prev = tracker->current;
	tracker->current.value = reading;
	tracker->current.ticks = get_real_ticks();
}

/**
 * Generates an output based on PID system.
 * Ki and Kd will be on halt on first call.
 */
int pid_output(PID* tracker, int target){
	if(tracker->prev.ticks == 0 || tracker->current.ticks == 0)
	{
		return target * tracker->kp;
	}
	else
	{
		tracker->proportion = target;
		tracker->integral += (target - tracker->current.value) * (get_real_ticks() - tracker->current.ticks + 1);
		tracker->derivative = (tracker->prev.value - tracker->current.value) / (tracker->prev.ticks - tracker->current.ticks);
		return tracker->proportion * tracker->kp + tracker->integral * tracker->ki + tracker->derivative * tracker->kd;
	}
}
