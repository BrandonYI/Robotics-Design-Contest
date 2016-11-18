#include "main.h"

#define MOTOR_LEFT MOTOR3
#define MOTOR_RIGHT MOTOR1

int main()
{
	//sys
	Encoder left_encoder;
	Encoder right_encoder;
	PID left_pid;
	PID right_pid;
	//init
	ticks_init();
	uart_init(COM3, 115200);
	motor_init(143, 10000, 0);
	tft_init(2, BLACK, WHITE);
	encoder_init(&left_encoder, ENCODER_LEFT);
	encoder_init(&right_encoder, ENCODER_RIGHT);
	pid_init(&left_pid, 30, 0.03, 0.01);
	pid_init(&right_pid, 30, 0.03, 0.01);
	uart_tx(COM3, "started.\r\n");
	while (1)
	{
		int out_left = pid_output(&left_pid, 20);
		int out_right = pid_output(&right_pid, 20);
		encoder_update(&left_encoder);
		encoder_update(&right_encoder);
		pid_sampling(&left_pid, get_encoder_velocity(&left_encoder));
		pid_sampling(&right_pid, get_encoder_velocity(&right_encoder));
		motor_control(MOTOR_LEFT, 0, out_left);
		motor_control(MOTOR_RIGHT, 0, out_right);
		tft_clear();
		tft_prints(10, 10, "%d", get_real_ticks());
		tft_prints(10, 20, "L: %d | %d", get_encoder_value(&left_encoder), left_encoder.rotations);
		tft_prints(10, 30, "R: %d | %d", get_encoder_value(&right_encoder), right_encoder.rotations);
		tft_prints(10, 50, "L_vel: %d", get_encoder_velocity(&left_encoder));
		tft_prints(10, 60, "L_OUT: %d", out_left);
		tft_prints(10, 80, "R_vel: %d", get_encoder_velocity(&right_encoder));
		tft_prints(10, 90, "R_OUT: %d", out_right);
		uart_tx(COM3, "vel: %d | %d ENCODER: %d | %d\r\n", get_encoder_velocity(&left_encoder), get_encoder_velocity(&right_encoder), get_encoder_value(&left_encoder), get_encoder_value(&right_encoder));
	}
}
