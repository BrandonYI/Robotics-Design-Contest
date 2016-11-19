#ifndef _ENCODER_H
#define _ENCODER_H

#include "ticks.h"

typedef enum {
	ENCODER_LEFT,
	ENCODER_RIGHT
}ENCODER_ID;

struct Encoder
{
	int id;
	int rotations;
	int prev;
	int prev_tick;
	int current;
	int current_tick;
	int prev_real;
	int current_real;
};
typedef struct Encoder Encoder;

void encoder_init(Encoder* encoder, ENCODER_ID id);
void encoder_update(Encoder* encoder);
int get_encoder_value(Encoder* encoder);
int get_encoder_velocity(Encoder* encoder);

#endif
