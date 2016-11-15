#ifndef _PID_H
#define _PID_H
#include "ticks.h"

struct Reading
{	
	int value;
	int ticks;
};
typedef struct Reading Reading;

struct PID
{
	double kp;
	double ki;
	double kd;
	int proportion;
	int integral;
	int derivative;
	Reading current;
	Reading prev;
};
typedef struct PID PID;

void pid_init(PID* tracker, double p, double i, double d);
void pid_sampling(PID* tracker, int reading);
int pid_output(PID* tracker, int target);

#endif
