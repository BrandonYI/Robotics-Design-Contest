#ifndef _ENCODER_H
#define _ENCODER_H
#include "ticks.h"

typedef enum {
	ENCODER_LEFT,
	ENCODER_RIGHT
} ENCODER_ID;

struct Encoder
{
	int id;
	int prev;
	int current;
	int prev_ticks_real;
	int current_ticks_real;
	int prev_ticks;
	int current_ticks;
	int rotations;
};
typedef struct Encoder Encoder;

void encoder_init(Encoder* encoder, ENCODER_ID id);
void encoder_update(Encoder* encoder);
int get_encoder_value(Encoder* encoder);
int get_encoder_velocity(Encoder* encoder);

#endif
