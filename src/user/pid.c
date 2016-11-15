#include "pid.h"

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

void pid_sampling(PID* tracker, int reading){
	Reading current = tracker->current;
	tracker->prev = tracker->current;
	tracker->current.value = reading;
	tracker->current.ticks = get_real_ticks();
}

int pid_output(PID* tracker, int target){
	if(tracker->prev.ticks == 0 || tracker->current.ticks == 0){
		return target * tracker->kp;
	} else{
		tracker->proportion = target;
		tracker->integral += (target - tracker->current.value) * (get_real_ticks() - tracker->current.ticks);
		tracker->derivative = (target - tracker->current.value) / (tracker->prev.ticks - tracker->current.ticks);
		return tracker->proportion * tracker->kp + tracker->integral * tracker->ki + tracker->derivative * tracker->kd;
	}
}
