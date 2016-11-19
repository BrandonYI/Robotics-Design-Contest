#include "main.h"
#define MOTOR_LEFT MOTOR3
#define MOTOR_RIGHT MOTOR1
#define clamp(val,min,max) val < min ? min : val > max ? max : val

//ROBOT
const int IS_SHOOTER_ROBOT = 1;
//Ticks
int last_tick_call = 0;
//PID
int out_left = 0;
int out_right = 0;
int target_velocity = 0;
int left_target_velocity = 0;
int right_target_velocity = 0;
int left_motor_magnitude = 0;
int right_motor_magnitude = 0;
Encoder left_encoder;
Encoder right_encoder;
PID left_pid;
PID right_pid;
//BLUETOOTH
const int MAXBUFFER = 10; //max buffer size for smartcar bluetooth incoming message
char buffer[MAXBUFFER] = {0}; //stores current input chars
int buffer_index = 0;
int bool_command_finish = 0;
int manual_mode = 0;
int release_time = 0;
//CCD
int ccdTime = 0; //for comparing with get_real_ticks() to know if its time to refresh ccd data
int ccd_rate = 50; //ccd update ms
const int MOVEMENT_SENS = 8; //64+-MOVEMENT_SENS before adjusting smartcar drive angle
//servo positions & speed
u16 speed_indic[3] = {RGB888TO565(0xC72929), RGB888TO565(0xFFC72C), RGB888TO565(0x40CA77)};
const int RIGHTMOST = 930;
const int LEFTMOST = 1090;
const int CENTER = 1010;
u16 servo_pos = 750;
u8 speed = 20;

void buffer_clear() {
    buffer_index = 0;
    uart_tx(COM3, "\n >>> ");
}
void reset_control() {
	pid_init(&left_pid, 0, 0, 0);
	pid_init(&right_pid, 0, 0, 0);
	left_motor_magnitude = 0;
	right_motor_magnitude = 0;
}
void uart_listener(const u8 byte) {
	buffer[buffer_index++] = byte;
	buffer[buffer_index] = '\0';
	uart_tx(COM3, "%c", byte);
	/******************************Commands**************************************/
	if (byte == '.') {
		bool_command_finish = 1;
	} else if (byte == 'x') {	//If you make a typo, press x to reset buffer
		buffer_clear();
	} else if (byte == 'q') {	//Stop Robot
		reset_control();
		buffer_clear();
	} else if (strstr(buffer, "pid")) {  //Check PID values
		uart_tx(COM3, "\n Left  p:%.2f i:%.2f d:%.2f", left_pid.kp, left_pid.ki, left_pid.kd);
		uart_tx(COM3, "\n Right p:%.2f i:%.2f d:%.2f", right_pid.kp, right_pid.ki, right_pid.kd);
		buffer_clear();
	} else if (strstr(buffer, "check")){
		uart_tx(COM3, "\n VEL: %d | %d", get_encoder_velocity(&left_encoder), get_encoder_velocity(&right_encoder));
		uart_tx(COM3, "\n PID OUT: %d | %d", out_left, out_right);
		uart_tx(COM3, "\n MAGNITUDE: %d | %d", left_motor_magnitude, right_motor_magnitude);
		uart_tx(COM3, "\n PROPORTION: %d | %d", left_pid.proportion, right_pid.proportion);
		uart_tx(COM3, "\n DERIVATIVE: %d | %d", left_pid.derivative, right_pid.derivative);
		uart_tx(COM3, "\n INTEGRAL: %d | %d", left_pid.integral, right_pid.integral);
		buffer_clear();
	} else if(strstr(buffer, "manual")){
		manual_mode = !manual_mode;
		buffer_clear();
	}
	/******************************MANUAL MODE********************************/
	if(manual_mode){
		release_time += get_real_ticks() - last_tick_call;
		reset_control(); //Calculate Appropriate Magnitudes Through PID
		if(byte == 'w'){
			left_motor_magnitude = right_motor_magnitude = 40;
			release_time = 0;
		} else if(byte == 'a'){
			right_motor_magnitude = 40;
			release_time = 0;
		} else if(byte == 's'){
			left_motor_magnitude = right_motor_magnitude = -40;
			release_time = 0;
		} else if(byte == 'd'){
			left_motor_magnitude = 40;
			release_time = 0;
		}
		if (release_time > 500) {
			left_motor_magnitude = right_motor_magnitude = 0;
		}
		buffer_clear();
	}
}
void bluetooth_handler() {
/************************Process Buffer*****************************/
    bool_command_finish = 0;
    char *cmdptr = strchr(buffer, ':');    //Locate ptr where the char : is first found
    if (cmdptr == NULL) {  //Account for invalid commands so it doesn't get stuck
        uart_tx(COM3, "\n invalid command");
        buffer_clear();
        return;
    }
    char *valptr = cmdptr + 1;
    char *idptr = cmdptr - 1;
    int val = strtol(valptr, NULL, 10); //Obtain Value
    *cmdptr = '\0';
    int id = strtol(idptr, NULL, 10); //Obtain ID
    *idptr = '\0';
/*********************Commands Post Processing**********************/
    if (strstr(buffer, "p") || strstr(buffer, "i") || strstr(buffer, "d")) {
        double dval = val/100.0;
        char k_id = buffer[0];
        switch (id) {
            case 0: //left
            	pid_update(&left_pid, dval, k_id);
             	uart_tx(COM3, "\n set left %c val to % .2f", k_id , dval);
             	break;
            case 1: //Right
            	pid_update(&right_pid, dval, k_id);
             	uart_tx(COM3, "\n set right %c val to % .2f", k_id , dval);
             	break;
            case 2: //Both
                pid_update(&left_pid, dval, k_id);
            	pid_update(&right_pid, dval, k_id);
             	uart_tx(COM3, "\n set both %c val to % .2f", k_id, dval);
             	break;
            }
    } else if (strstr(buffer, "targe")) {
        uart_tx(COM3, "\n set target to %d", val);
        target_velocity = val;
    }
    buffer_clear();
}
int main() {
	//init
	ticks_init();
	uart_init(COM3, 115200);
	uart_interrupt_init(COM3, &uart_listener); //com port, function
    uart_tx(COM3, "initialize\n");
    uart_tx(COM3, ">>> ");
	motor_init(143, 10000, 0);
	tft_init(2, BLACK, WHITE);
	if (IS_SHOOTER_ROBOT == 1) {
		encoder_init(&left_encoder, ENCODER_LEFT);
		encoder_init(&right_encoder, ENCODER_RIGHT);
		pid_init(&left_pid, 0, 0, 0);
		pid_init(&right_pid, 0, 0, 0);
	}
	if (IS_SHOOTER_ROBOT == 0) { //cant use servo if you're using encoders because both use timer 3
        servo_init(143, 10000, 0);
    }
    if(IS_SHOOTER_ROBOT) {
		while (1)
		{
			if(last_tick_call != get_real_ticks()){
				last_tick_call = get_real_ticks();

				if(target_velocity != 0) {
					//PID Update
					out_left = pid_output(&left_pid, target_velocity);
					out_right = pid_output(&right_pid, target_velocity);
					/*int out_left = pid_output(&left_pid, clamp(target_velocity * get_second_ticks() / 4, -target_velocity, target_velocity));
					int out_right = pid_output(&right_pid, clamp(target_velocity * get_second_ticks() / 4, -target_velocity, target_velocity));*/
					left_motor_magnitude += out_left;
					right_motor_magnitude += out_right;
					encoder_update(&left_encoder);
					encoder_update(&right_encoder);
					pid_sampling(&left_pid, get_encoder_velocity(&left_encoder));
					pid_sampling(&right_pid, get_encoder_velocity(&right_encoder));
					motor_control(MOTOR_LEFT, (left_motor_magnitude > 0 ? 0 : 1), left_motor_magnitude);
					motor_control(MOTOR_RIGHT, (right_motor_magnitude > 0 ? 0 : 1), right_motor_magnitude);
				}
				//Bluetooth Checking
				if(bool_command_finish){
					bluetooth_handler();
				}
				//Try To Fix TFT Flicker Problem
				tft_clear();
				tft_prints(10, 10, "%d", get_real_ticks());
				tft_prints(10, 20, "L: %d | %d", get_encoder_value(&left_encoder), left_encoder.rotations);
				tft_prints(10, 30, "R: %d | %d", get_encoder_value(&right_encoder), right_encoder.rotations);
				tft_prints(10, 50, "L_vel: %d", get_encoder_velocity(&left_encoder));
				tft_prints(10, 60, "L_OUT: %d" , out_left);
				tft_prints(10, 70, "L_MAG: %d" , left_motor_magnitude);
				tft_prints(10, 90, "R_vel: %d", get_encoder_velocity(&right_encoder));
				tft_prints(10, 100, "R_OUT: %d", out_right);
				tft_prints(10, 110, "R_MAG: %d", right_motor_magnitude);
			}
		}
	}
	/*********************************SMART CAR CODE********************************************/
	else{
		while (1) {
            tft_prints(0, 0, "calibrate ccd on dark area");
            tft_prints(10, 10, "any btn to continue");
            if (read_button(BUTTON1) == 0 || read_button(BUTTON2) == 0 || read_button(BUTTON3) == 0) {
                linear_ccd_read();
                int k;
                for (k = 0; k < 128; k++) {
                    calibrateCCD[k] = 0 - linear_ccd_buffer1[k];
                }
                break;
            }
        }
        tft_clear();
        int avg = 0; //avg of left and right edge
        int leftEdge = 0, rightEdge = 0;
        while (1) {
            if (read_button(BUTTON1) == 0 && servo_pos < LEFTMOST) {
                servo_pos += speed;
                tft_fill_area(46, 72, 25, 12, BLACK);
                tft_prints(46, 72, "%d", servo_pos);
                servo_control(SERVO1, servo_pos);
            }
            if (read_button(BUTTON3) == 0 && servo_pos > RIGHTMOST) {
                servo_pos -= speed;
                tft_fill_area(46, 72, 25, 12, BLACK);
                tft_prints(46, 72, "%d", servo_pos);
                servo_control(SERVO1, servo_pos);
            }
            if (get_real_ticks() - ccdTime >= ccd_rate) { //Update by CCD Rate
                ccdTime = get_real_ticks();
                int k;
                for (k = 0; k < 128; k++) { //Clear CCD Screen
                    tft_put_pixel(k, 159 - linear_ccd_buffer1[k], BLACK);
                    tft_put_pixel(k, 159 - medianCCD[k], BLACK);
                    tft_put_pixel(k, 159 - schmittCCD[k], BLACK);
                }
                if (leftEdge != -1) {
                    drawLine(leftEdge, 0, BLACK);
                }
                if (rightEdge != -1) {
                    drawLine(rightEdge, 0, BLACK);
                }
                drawLine(avg, 0, BLACK);
                linear_ccd_read();
                //calibrate_ccd();
                runMedianFilter();
                runSchmitt();

                leftEdge = -1;
                rightEdge = 128;
                calculateSumPrefix(&leftEdge, &rightEdge);

                //tft_fill_area(50, 50, 60, 20, BLACK);
                if (leftEdge != -1) {
                    drawLine(leftEdge, 0, GREEN);
                }
                if (rightEdge != 128) {
                    drawLine(rightEdge, 0, GREEN);
                }

                avg = (leftEdge + rightEdge) / 2;
                drawLine(avg, 0, RED);

                tft_fill_area(38, 50, 80, 20, BLACK);

                drawLine(64 - MOVEMENT_SENS, 0, WHITE);
                drawLine(64 + MOVEMENT_SENS, 0, WHITE);

                if (avg < 64 - MOVEMENT_SENS) {
                    tft_prints(38, 50, "L %d%", clamp(CENTER + (LEFTMOST - CENTER) * ((64 - avg) / 20.0), RIGHTMOST, LEFTMOST));
                    servo_control(SERVO1, clamp(CENTER + (LEFTMOST - CENTER) * ((64 - avg) / 20.0), RIGHTMOST, LEFTMOST));
                } else if (avg > 64 + MOVEMENT_SENS) {
                    tft_prints(38, 50, "R %d%", clamp(CENTER - (CENTER - RIGHTMOST) * ((avg - 64) / 20.0), RIGHTMOST, LEFTMOST));
                    servo_control(SERVO1, clamp(CENTER - (CENTER - RIGHTMOST) * ((avg - 64) / 20.0), RIGHTMOST, LEFTMOST));
                } else {
                    servo_control(SERVO1, CENTER);
                    tft_prints(50, 50, "%d", CENTER);
                }

                for (k = 0; k < 128; k++) { //Add CCD onto Screen
                    tft_put_pixel(k, 159 - linear_ccd_buffer1[k], RED);
                    tft_put_pixel(k, 159 - schmittCCD[k], GREEN);
                    tft_put_pixel(k, 159 - medianCCD[k], WHITE);
                }
                int h;
                for (h = 0; h < 159; h += 20) { //draw label
                    tft_prints(2, h, "%d", 159 - h);
                }
            }
        }
	}
}
