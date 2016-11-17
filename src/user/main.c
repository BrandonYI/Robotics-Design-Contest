#include "main.h"
#define abs(x) x < 0 ? -x : x
#define clamp(val,min,max) val < min ? min : val > max ? max : val
const int IS_SHOOTER_ROBOT = 1; //to decide which while loop to use

//////shooter robot variables
/*
*      				curAB = 00 		curAB = 01		curAB = 10		curAB = 11
*oldAB = 00		NO CHANGE			-1 CCW				1 CW					UNKNOWN
*oldAB = 01		1 CW					NO CHANGE			UNKNOWN				-1 CCW
*oldAB = 10		-1 CCW				UNKNOWN				NO CHANGE			1 CW
*oldAB = 11		NO CHANGE			1 CW	S				-1 CCW				UNKNOWN
*/
/*int encoderArr[4][4] = {
    {0, -1, 1, 0},
    {1, 0, 0, -1},
    {-1, 0, 0, 1},	
    {0, 1, -1, 0}
};*/
u32 curtime = 0;
//Motor
int left_pid_output = 0;
int right_pid_output = 0;
const int MOTOR_MAGNITUDE_LIM = 8000;
//Encoder
int target_enc_vel = 0; //target encoder velocity in ticks/ms
double encoder_constant = 0; //Encoder offset
PID left_pid;
PID right_pid;
Encoder left_encoder;
Encoder right_encoder;

/************************carrier robot (smartcar)***************************/
//bluetooth
const int MAXBUFFER = 100; //max buffer size for smartcar bluetooth incoming message
char buffer[MAXBUFFER] = {0}; //stores current input chars
int buffer_index = 0;
int bool_command_finish = 0;

//ccd
int ccdTime = 0; //for comparing with get_real_ticks() to know if its time to refresh ccd data
int ccd_rate = 50; //ccd update ms

const int CCD_THRESH = 100; //binary classifier ccd value [0 - 159]
const int WINDOWSIZE = 10; //median filter windowsize
const int MOVEMENT_SENS = 8; //64+-MOVEMENT_SENS before adjusting smartcar drive angle

u8 medianCCD[128] = {0}; //stores medians
u8 schmittCCD[128] = {0}; //stores 0 or 1
int sumDiffCCD[128] = {0}; //stores sum prefix
int calibrateCCD[128] = {0}; //stores initial calibration offset

//servo positions & speed
u16 speed_indic[3] = {RGB888TO565(0xC72929), RGB888TO565(0xFFC72C), RGB888TO565(0x40CA77)};
const int RIGHTMOST = 930;
const int LEFTMOST = 1090;
const int CENTER = 1010;
u16 servo_pos = 750;
u8 speed = 20;

/*************************misc functions*************************/
void count_ticks(){
	double ratio = 0;
	uart_tx(COM3, "\n Left Motor Encoder Velocity: %d", get_encoder_velocity(&left_encoder));
	uart_tx(COM3, "\n Right Motor Encoder Velocity: %d", get_encoder_velocity(&right_encoder));
	ratio = (double)get_encoder_velocity(&right_encoder) / (double)get_encoder_velocity(&left_encoder);
	uart_tx(COM3, "\n Ratio: %.2f", ratio);
}

void drawLine(int val, int isHorizontal, u16 color) {
    int k;
    if (isHorizontal) {
        for (k = 0; k < 159; k++)
            tft_put_pixel(k, val, color);
    } else {
        for (k = 0; k < 159; k++) {
            tft_put_pixel(val, k, color);
        }
    }
}

int bitStringToInt(int a, int b) {
    if (a == 1 && b == 1) {
        return 3;
    } else if (a == 1 && b == 0) {
        return 2;
    } else if (a == 0 && b == 1) {
        return 1;
    } else if (a == 0 && b == 0) {
        return 0;
    }
}

/*************************carrier robot (smartcar)*************************/
void buffer_clear() {
    buffer_index = 0;
    uart_tx(COM3, "\n >>> ");
}

void change_speed(void) {
    static u8 count = 0;
    count++;
    speed = 20; //case 0
    switch (count % 3) {
        case 2: //speed = 60
            speed += 20;
        case 1: //speed = 40
            speed += 20;
    }
    tft_fill_area(72, 72, 12, 12, speed_indic[count % 3]);
}

void use_servo(long value, long id) {
    if (value < LEFTMOST && value > RIGHTMOST) {
        servo_control((SERVO_ID) id, value);
    } else {
        uart_tx(COM3, "\n servo value is out of range");
    }
}

void uart_listener(const u8 byte) {
    buffer[buffer_index++] = byte;
    buffer[buffer_index] = '\0';
    uart_tx(COM3, "%c", byte);
    if (byte == '.') {
        bool_command_finish = 1;
    }
    if (byte == 'x') {	//If you make a typo, press x to reset buffer
        buffer_clear();
    }
    if (byte == 'q') {	//Stop Robot
    	pid_init(&left_pid, 0, 0, 0);
    	pid_init(&right_pid, 0, 0, 0);
    	buffer_clear();
    }
    if (strstr(buffer, "pid")) {  //Check PID values
    	uart_tx(COM3, "\n Left p:%.2f i:%.2f d:%.2f", left_pid.kp, left_pid.ki, left_pid.kd);
    	uart_tx(COM3, "\n Right p:%.2f i:%.2f d:%.2f", right_pid.kp, right_pid.ki, right_pid.kd);
    	buffer_clear();
	}
	if (strstr(buffer, "count")) {
		count_ticks();
		buffer_clear();
	}
}

float getMedian(const int a[]) {
    int arr[WINDOWSIZE] = {0};
    for (int k = 0; k < WINDOWSIZE; k++) { //copy into temporary array
        arr[k] = a[k];
    }
    int key, i, j;
    for (i = 2; i < WINDOWSIZE; i++) { //selection sort
        key = arr[i];
        j = i - 1;
        while (j > 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            --j;
        }
        arr[j + 1] = key;
    }
    return (WINDOWSIZE % 2 == 1 ? arr[WINDOWSIZE / 2] : (arr[WINDOWSIZE / 2 - 1] + arr[WINDOWSIZE / 2]) / 2);
}

void runSchmitt() {
    int k;
    for (k = 0; k < 128; k++) {
        schmittCCD[k] = (medianCCD[k] < CCD_THRESH) ? 1 : 158;
    }
}

void calculateSumPrefix(int *leftCandidate, int *rightCandidate) {
    int k;
    for (k = 0; k < 128 - 1; k++) {
        sumDiffCCD[k] = schmittCCD[k] - schmittCCD[k + 1];
    }

    for (k = 0; k < 127; k++) { //negative value means close to left edge
        if (sumDiffCCD[k] < 0) {
            *leftCandidate = k;
            break;
        }
    }

    for (k = 126; k >= 0; k--) {
        if (sumDiffCCD[k] > 0) {
            *rightCandidate = k;
            break;
        }
    }
}

void calibrate_ccd() {
    int k;
    for (k = 0; k < 128; k++) {
        linear_ccd_buffer1[k] += calibrateCCD[k];
        linear_ccd_buffer1[k] = clamp(linear_ccd_buffer1[k], 0, 128);
    }
}

void runMedianFilter() {
    int curWindow[WINDOWSIZE] = {0};
    int indexOfOldest = 0;
    for (int k = 0; k < WINDOWSIZE; k++) {	//initialize the window
        curWindow[k] = linear_ccd_buffer1[k];
    }
    //int firstMaxMedianIndex = 0;
    //int lastMaxMedianIndex = 0;
    for (int j = 0; j <= WINDOWSIZE / 2; j++) {
        medianCCD[j] = getMedian(curWindow);
    }

    for (int k = WINDOWSIZE; k < 128; k++) {
        curWindow[indexOfOldest] = linear_ccd_buffer1[k];
        indexOfOldest++;
        indexOfOldest %= WINDOWSIZE;
        medianCCD[k - WINDOWSIZE / 2] = getMedian(curWindow);
        /*
        if (medianCCD[k-WINDOWSIZE/2] >= medianCCD[lastMaxMedianIndex]) {
        lastMaxMedianIndex = k - WINDOWSIZE/2;
        if (medianCCD[k-WINDOWSIZE/2] > medianCCD[firstMaxMedianIndex])
        firstMaxMedianIndex = k - WINDOWSIZE/2;
        }
        */
    }
    for (int k = 128 - WINDOWSIZE; k < 128; k++) {
        medianCCD[k] = getMedian(curWindow);
    }
}

void bluetooth_handler() {
        /************************Process Buffer***************************/
    if (bool_command_finish) {
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
        /*uart_tx(COM3, "\n COMMAND: %s   ", buffer);
        uart_tx(COM3, "ID: %ld   ", id);
        uart_tx(COM3, "VAL: %ld", val);*/
        /************************Commands**********************************/
        if (strstr(buffer, "led")) { //if detect substring led(strstr returns a pointer)
            led_toggle((LED_ID)id); //LED
            uart_tx(COM3, "\n LED %ld is %d", id, val);
        } else if (strstr(buffer, "motor")) { //if detect substring motor
            motor_control((MOTOR_ID) id, (val > 0 ? 0 : 1), val); //MOTOR
            uart_tx(COM3, "\n motor %ld is %d", id, val);
        } else if (strstr(buffer, "servo")) { //if detect substring servo
            use_servo(val, id); //SERVO
            uart_tx(COM3, "\n servo %ld is %d", id, val);
        } else if (strstr(buffer, "pneumatic")) {
            pneumatic_control((PNEUMATIC_ID) id, val); //PNEUMATIC
            uart_tx(COM3, "\n pneumatic %ld is %d", id, val);
        } else if (strstr(buffer, "p") || strstr(buffer, "i") || strstr(buffer, "d")) {
            double dval = val/100.0;
            char k_id = buffer[0];
            switch (id) {
                case 0: //left
                	pid_update(&left_pid, dval, k_id);
                 	uart_tx(COM3, "\n set left %c val to % f", k_id , dval);
                 	break;
                case 1: //Right
                	pid_update(&right_pid, dval, k_id);
                 	uart_tx(COM3, "\n set right %c val to % f", k_id , dval);
                 	break;
                case 2: //Both
                    pid_update(&left_pid, dval, k_id);
                	pid_update(&right_pid, dval, k_id);
                 	uart_tx(COM3, "\n set both %c val to % f", k_id, dval);
                 	break;
	            }
        } else if (strstr(buffer, "targe")) {
            uart_tx(COM3, "\n set target to %d", val);
            target_enc_vel = val;
            ticks_reset();
        } else if (strstr(buffer, "offse")) {
        	double dval = val/100.0;
            uart_tx(COM3, "\n set encoder_constant to %.2f", dval);
            encoder_constant = val;
        }
        buffer_clear();
    }
}

/*************************shooter robot *************************/
void PID_motor_update() {
	encoder_update(&left_encoder);
	pid_sampling(&left_pid, get_encoder_velocity(&left_encoder));
	left_pid_output = pid_output(&left_pid, clamp(target_enc_vel * get_second_ticks() / 4, -target_enc_vel, target_enc_vel));
	//left_pid_output = pid_output(&left_pid, target_enc_vel);
	motor_control(MOTOR3, (left_pid_output > 0 ? 0 : 1), abs(left_pid_output));

	encoder_update(&right_encoder);
	pid_sampling(&right_pid, get_encoder_velocity(&right_encoder));
	right_pid_output = pid_output(&right_pid, clamp(target_enc_vel * get_second_ticks() / 4, -target_enc_vel, target_enc_vel));
	//right_pid_output = pid_output(&right_pid, target_enc_vel);
	motor_control(MOTOR1, (right_pid_output > 0 ? 0 : 1), abs(right_pid_output));
}

int main() {
    led_init();
    gpio_init();
    ticks_init();
    linear_ccd_init();
    adc_init();
    button_init();
    set_keydown_listener(BUTTON2, &change_speed);
    tft_init(2, BLACK, WHITE);
    uart_init(COM3, 115200);
    uart_interrupt_init(COM3, &uart_listener); //com port, function
    uart_tx(COM3, "initialize\n");
    uart_tx(COM3, ">>> ");
    motor_init(143, 10000, 0);
    if (IS_SHOOTER_ROBOT == 0) { //cant use servo if you're using encoders because both use timer 3
        servo_init(143, 10000, 0);
    }
    if (IS_SHOOTER_ROBOT == 1) {
        encoder_init(&left_encoder,ENCODER_LEFT);
        encoder_init(&right_encoder,ENCODER_RIGHT);
        pid_init(&left_pid, 20, .01, 0);
        pid_init(&right_pid, 20, .01, 0);
        while (1) {
        	if(curtime != get_real_ticks()){
        		curtime = get_real_ticks();
	            bluetooth_handler();
	            PID_motor_update(); //PID Motor Update
	            tft_clear();
	            tft_prints(10, 10, "Left: %d", get_encoder_value(&left_encoder));
	            tft_prints(10, 20, "Right: %d", get_encoder_value(&right_encoder));
	            tft_prints(10, 30, "Time: %d", get_real_ticks());
	            tft_prints(10, 40, "Lout: %d   Rout:  %d", left_pid_output, right_pid_output);
	            //tft_prints(10, 50, "Lmag: %d   Rmag:  %d", left_motor_magnitude, right_motor_magnitude);
	            tft_prints(10, 60, "Lvel: %d    target: %d", get_encoder_velocity(&left_encoder), target_enc_vel);
	            tft_prints(10, 70, "Rvel: %d    target: %d", get_encoder_velocity(&right_encoder), target_enc_vel);
	            tft_prints(10, 80, "Lp: %d  Rp: %d", left_pid.proportion, right_pid.proportion);
	            tft_prints(10, 90, "Ld: %d  Rd: %d", left_pid.derivative, right_pid.derivative);
	            tft_prints(10, 90, "Li: %d        Ri: %d", left_pid.integral, right_pid.integral);
	        }
        }
    } else { // is smartcar code
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
