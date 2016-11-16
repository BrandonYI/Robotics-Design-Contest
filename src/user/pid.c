#include "pid.h"
#define clamp(val,min,max) val < min ? min : val > max ? max : val
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

void pid_update(PID*tracker, double dval, char k_id){
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
	//tracker->current.value = clamp(tracker->current.value, 0 , 1000);
	tracker->current.ticks = get_real_ticks();
}

int pid_output(PID* tracker, int target){
	if(tracker->prev.ticks == 0 || tracker->current.ticks == 0){
		return target * tracker->kp;
	} else{
		tracker->proportion = target;
		tracker->integral += (target - tracker->current.value) * (get_real_ticks() - tracker->current.ticks + 1);
		tracker->derivative = (tracker->prev.value - tracker->current.value) / (tracker->prev.ticks - tracker->current.ticks);
		/*double error = tracker->current.value - target; //Target needs to be ticks per cycle
		tracker->proportion = error;
		tracker->derivative = (tracker->prev_error - error) / (tracker->prev.ticks - tracker->current.ticks);
		tracker->integral += (error * (get_real_ticks() - tracker->current.ticks + 1));
		tracker->prev_error = error;*/
		return tracker->proportion * tracker->kp + tracker->integral * tracker->ki + tracker->derivative * tracker->kd;
	}
}

//TIM3: left wheel  TIM4: Right wheel
/*int pid_actual(PID* tracker) {
    int change = 0;
    if (tracker->prev.value - tracker->current.value > 25000) { //jumped gap from 65534 ... 65535 ... 0 ... 1 ... 2
        change += 65535;
    } else if (tracker->prev.value - tracker->current.value < -25000) { //jumped gap from 2 ... 1 ... 0 ... 65535 ... 65534
        change -= 65535;
    }
    //return (change + tracker->current.value - tracker->prev.value) / (tracker->current.ticks - tracker->prev.ticks);
    return change += (tracker->current.value - tracker->prev.value);
}
*/
