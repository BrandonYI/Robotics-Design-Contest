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
	tracker->prev_error = 0;
}

void pid_update(PID* tracker, double dval, char k_id){
	switch(k_id){
		case'p':
			tracker->kp = dval;
			break;
		case'i':
			tracker->ki = dval;
			break;
		case'd':
			tracker->kd = dval;
			break;
 	}
}

void pid_sampling(PID* tracker, int reading){
	tracker->prev = tracker->current;
	tracker->current.value = reading;
	tracker->current.ticks = get_real_ticks();
}

int pid_output(PID* tracker, int target){
	if(tracker->prev.ticks == 0 || tracker->current.ticks == 0){
		return target * tracker->kp;
	} else{
		tracker->proportion = target;
		tracker->integral += (target - tracker->current.value) * (get_real_ticks() - tracker->current.ticks + 1);
		tracker->derivative = (tracker->prev.value - tracker->current.value) / (tracker->prev.ticks - tracker->current.ticks);
		/*double error = target - tracker->current.value; //Target needs to be ticks per cycle
		tracker->proportion = error;
		tracker->derivative = (error - tracker->prev_error) / (tracker->current.ticks - tracker->prev.ticks + 1);
		tracker->integral += (error * (get_real_ticks() - tracker->current.ticks + 1));
		tracker->prev_error = error;*/
		return (tracker->proportion * tracker->kp) + (tracker->integral * tracker->ki) + (tracker->derivative * tracker->kd);
	}
}
