#include "main.h"

const int IS_SHOOTER_ROBOT = 1; //to decide which while loop to use

////prototypes

void use_motor(long, long);
void use_pneumatic(long, long);
void use_led(long, long);

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
//motor 
double left_motor_magnitude = 0;
double right_motor_magnitude = 0;

//PID Control

long target_enc_vel = 40; //target encoder velocity in ticks/ms
u32 lastEncoderReadTime = 0;
u32 timePassed = 0;

long old_enc_leftX = 0;
long old_enc_rightX = 0;
long enc_leftV = 0; //left encoder angular velocity
long enc_rightV = 0;
int old_enc_Lerror = 0;
int old_enc_Rerror = 0;

//PID
double left_proportion = 0;
double left_derivative = 0;
double left_integral = 0;
double right_proportion = 0;
double right_derivative = 0;
double right_integral = 0;

//Constants for Tuning PID
double left_kp = 0;
double left_ki = 0;
double left_kd = 0;

double right_kp = 0;
double right_ki = 0;
double right_kd = 0;


//////carrier robot (smartcar)
//bluetooth
const int MAXBUFFER = 100; //max buffer size for smartcar bluetooth incoming message
char buffer[MAXBUFFER] = {0}; //stores current input chars
char manual_buffer[MAXBUFFER] = {0};
char curKey = '\0';

int buffer_index = 0;
int bool_command_finish = 0;
int bool_need_clear_buffer = 0;
int timeSinceLastCommand = 0; //to check if key no longer pressed
int curTime = 0; //for comparing with timeSinceLastCommand

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
float clamp(float val, int min, int max) {
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

int abs(int a) {
    return a<0? -a: a;
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

int bitStringToInt(int a, int b) {  //What is this for? This is super inefficient
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
    if (bool_need_clear_buffer) {
        bool_need_clear_buffer = 0;
        buffer_index = 0;
        uart_tx(COM3, "\n>>> ");
    }
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
    if(value<LEFTMOST&&value>RIGHTMOST) {
        servo_control((SERVO_ID)id, value);
    } else {
        uart_tx(COM3, "servo value is out of range\n");
    }
}

void uart_listener(const u8 byte) {
    curTime = get_real_ticks();
    timeSinceLastCommand = 0;
    buffer[buffer_index++] = byte;
    buffer[buffer_index] = '\0';
    uart_tx(COM3, "%c", byte);
    if (byte == '.') {
        bool_command_finish = 1;
        buffer_index = 0;
    }
    if(byte == 'x') {	//If you make a typo, press x to reset buffer
        bool_need_clear_buffer = 1;
        buffer_index = 0;
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
        while (j>0 && arr[j] > key) {
            arr[j+1] = arr[j];
            --j;
        }
        arr[j+1] = key;
    }
    return (WINDOWSIZE%2==1? arr[WINDOWSIZE/2] : (arr[WINDOWSIZE/2-1]+arr[WINDOWSIZE/2])/2);
}
void runSchmitt() {
    int k;
    for (k = 0; k < 128; k++) {
        schmittCCD[k] = (medianCCD[k] < CCD_THRESH)? 1: 158;
    }
}

void calculateSumPrefix(int* leftCandidate, int* rightCandidate) {
    int k;
    for (k = 0; k < 128-1; k++) {
        sumDiffCCD[k] = schmittCCD[k]-schmittCCD[k+1];
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
    //initialize the window
    for (int k = 0; k < WINDOWSIZE; k++) {
        curWindow[k] = linear_ccd_buffer1[k];
    }
    //int firstMaxMedianIndex = 0;
    //int lastMaxMedianIndex = 0;
    for (int j = 0; j <= WINDOWSIZE/2; j++) {
        medianCCD[j] = getMedian(curWindow);
    }

    for (int k = WINDOWSIZE; k < 128; k++) {
        curWindow[indexOfOldest] = linear_ccd_buffer1[k];
        indexOfOldest++;
        indexOfOldest%=WINDOWSIZE;
        medianCCD[k-WINDOWSIZE/2] = getMedian(curWindow);
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
    if (bool_command_finish) {
        bool_command_finish = 0;
        uart_tx(COM3, "\nCOMPLETE COMMAND: %s\n", buffer);

        char* cmdptr = strchr(buffer, ':');	//Locate ptr where the char : is first found
        if (cmdptr == NULL){
					uart_tx(COM3, "invalid command\n");
					uart_tx(COM3, ">>> ");					
					buffer_clear();
					return;
				}
			
				char* valptr = cmdptr + 1;
        char* idptr = cmdptr - 1;
        int val = strtol(valptr, NULL,10); //Obtain Value
        *cmdptr = '\0';
        int id = strtol(idptr, NULL,10); //Obtain ID
        *idptr = '\0';
        uart_tx(COM3, "COMMAND: %s   ", buffer);
        uart_tx(COM3, "ID: %ld   ", id);
        uart_tx(COM3, "VAL: %ld\n", val);

        if(strstr(buffer, "led")) { //if detect substring led(strstr returns a pointer)
            bool_need_clear_buffer = 1;
            use_led(val, id); //LED
        } else if(strstr(buffer,"motor")) { //if detect substring motor
            use_motor(val, id); //MOTOR
            uart_tx(COM3,"motor %ld is on \n", id);
        } else if(strstr(buffer,"servo")) { //if detect substring servo
            use_servo(val, id); //SERVO
            uart_tx(COM3, "servo %ld is on \n", id);
        } else if(strstr(buffer,"pneumatic")) {
            use_pneumatic(val, id); //PNEUMATIC
            uart_tx(COM3, "pneumatic %ld is on \n", id);
				} else if (strstr(buffer, "p")){ //have to type pp:1.
						if (id == 0){ //left
							uart_tx(COM3, "set left p val to %f \n", val/100.0);
							left_kp = val/100.0;
						} else if (id == 1){
							uart_tx(COM3, "set right p val to %f \n", val/100.0);
							right_kp = val/100.0;
						}		
				} else if (strstr(buffer,"i")){
						if (id == 0){ //left
							uart_tx(COM3, "set left i val to %f \n", val/100.0);
							left_ki = val/100.0;
						} else if (id == 1){
							uart_tx(COM3, "set right i val by %f \n", val/100.0);
							right_ki = val/100.0;
						}						
				} else if (strstr(buffer, "d")){
						if (id == 0){ //left
							uart_tx(COM3, "set left d val to %f \n", val/100.0);
							left_kd = val/100.0;
						} else if (id == 1){
							uart_tx(COM3, "set right d val to %f \n", val/100.0);
							right_kd = val/100.0;
						}		
				} else if (strstr(buffer, "targe")){
						uart_tx(COM3, "set target to %d", val);
						target_enc_vel = val;
				}
        bool_need_clear_buffer = 1;
    }
    buffer_clear();
}

/*************************shooter robot *************************/
void use_motor(long value, long id) {
    if(value<100 && value>0) {
        motor_control((MOTOR_ID)id, 1, value);
    } else {
        uart_tx(COM3, "motor value is out of range\n");
    }
}

void use_pneumatic(long value, long id) {
    pneumatic_control((PNEUMATIC_ID)id, value); //1 is on, others is off
}

void use_led(long value, long id) {
    if(value == 1) {
        led_on((LED_ID)id);
        uart_tx(COM3, "TURNED LED %ld ON\n", id);
    } else {
        led_off((LED_ID)id);
        uart_tx(COM3, "TURNED LED %ld OFF\n", id);
    }
}


double get_ang_vel(double change, u32 timePassed){
	return -change/timePassed;	//negative because we fucked up the wiring
}
//TIM3: left wheel
int get_left_enc_pos_change(int old_enc_pos){
	int change = 0;
	int cur_left_enc_pos = TIM3->CNT; //use long?
	if (old_enc_pos - cur_left_enc_pos > 25000){ //jumped gap from 65534 ... 65535 ... 0 ... 1 ... 2
		change += 65535;
	} else if (old_enc_pos - cur_left_enc_pos < - 25000){ //jumped gap from 2 ... 1 ... 0 ... 65535 ... 65534
		change -= 65535;
	}	
	change += (cur_left_enc_pos - old_enc_pos);
	return change;
}
//TIM4: right wheel
int get_right_enc_pos_change(int old_enc_pos){
	int change = 0;
	int cur_right_enc_pos = TIM4->CNT; //use long?
	if (old_enc_pos - cur_right_enc_pos > 25000){ //jumped gap from 65534 ... 65535 ... 0 ... 1 ... 2
		change += 65535;
	} else if (old_enc_pos - cur_right_enc_pos < - 25000){ //jumped gap from 2 ... 1 ... 0 ... 65535 ... 65534
		change -= 65535;
	}	
	change += (cur_right_enc_pos - old_enc_pos);
	return change;
}
void PID_motor_update(){
	timePassed = get_real_ticks() - lastEncoderReadTime;
	lastEncoderReadTime = get_real_ticks();

	int enc_Lerror = 0; //Error between target velocity and actual velocity
	int enc_Rerror = 0;
	
	/******************************left**************************************/
	enc_leftV = get_ang_vel( get_left_enc_pos_change(old_enc_leftX), timePassed); //Calculate the left velocity (pos - oldpos / time)
	enc_Lerror = target_enc_vel - enc_leftV; //Calculate the error
	
	left_proportion = enc_Lerror;  //Calculate Proportion
	left_derivative = enc_Lerror - old_enc_Lerror;  //Calculate Derivative
	left_integral += left_proportion;  //Calculate Integral

	left_motor_magnitude += left_proportion*left_kp + left_derivative*left_kd + left_integral*left_ki; //Speed = P + I + D
	left_motor_magnitude = clamp(left_motor_magnitude, 1, 5000); //Clamping
	motor_control(MOTOR1, 0, (int)left_motor_magnitude); //Use Speed
	old_enc_leftX = TIM3->CNT;  //Update Old Position
	old_enc_Lerror = enc_Lerror;  //Update Old Velocity Error

	/*****************************Right**************************************/
	enc_rightV = get_ang_vel( get_right_enc_pos_change(old_enc_rightX), timePassed);
	enc_Rerror = target_enc_vel - enc_rightV;

	right_proportion = enc_Rerror;
	right_derivative = enc_Rerror - old_enc_Rerror;
	right_integral += right_proportion;

	right_motor_magnitude += right_proportion*right_kp + right_derivative*right_kd + right_integral*right_ki;
	right_motor_magnitude = clamp(right_motor_magnitude, 1, 5000);
	motor_control(MOTOR3, 0, (int)right_motor_magnitude);
	old_enc_rightX = TIM4->CNT;
	old_enc_Rerror = enc_Rerror;
}
int main() {
    led_init();
    gpio_init();
    ticks_init();
    linear_ccd_init();
    adc_init();
    button_init();
    set_keydown_listener(BUTTON2, &change_speed);
	if (IS_SHOOTER_ROBOT == 0){ //cant use servo if you're using encoders because both use timer 3
		servo_init(143,10000,0);
	}
    tft_init(2, BLACK, WHITE);
    uart_init(COM3, 115200);
    uart_interrupt_init(COM3, &uart_listener); //com port, function
    uart_tx(COM3, "initialize\n");
		uart_tx(COM3, ">>> ");
    motor_init(143, 10000, 0);
    if (IS_SHOOTER_ROBOT == 1) { 
        init_encoder_left();
        init_encoder_right();
        while (1) {
        	PID_motor_update(); //PID Motor Update
        	bluetooth_handler();
					tft_clear();
					tft_prints(10, 10, "left: %d", TIM4->CNT);
					tft_prints(10, 20, "right: %d", TIM3->CNT);
					tft_prints(10, 30, "Lmotor mag: %f", left_motor_magnitude);
					tft_prints(10, 40, "Rmotor mag: %f", right_motor_magnitude);
					tft_prints(10, 50, "Lerror: %d", old_enc_Lerror);
					tft_prints(10, 60, "Rerror: %d", old_enc_Rerror);
					tft_prints(10, 70, "L cur_vel:%d  target:%d", enc_leftV, target_enc_vel);	
					tft_prints(10, 80, "R cur_vel:%d  target:%d", enc_rightV, target_enc_vel);
					tft_prints(10, 90, "L_p:%.2f  R_p:%.2f", left_kp, right_kp);
					tft_prints(10, 100,"L_i:%.2f   R_i:%.2f", left_ki, right_ki);
					tft_prints(10, 110,"L_d:%.2f  R_d:%.2f", left_kd, right_kd);
				}

    } else { // is smartcar code
        while(1) {
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
            //motor_control(0, 0, 50);
            //motor_control(1, 0, 50);
            //motor_control(2, 0, 50);
        }
        tft_clear();
		int avg = 0; //avg of left and right edge
		int leftEdge = 0, rightEdge = 0;
        while(1) {
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
                    tft_put_pixel(k, 159-linear_ccd_buffer1[k], BLACK);
                    tft_put_pixel(k, 159-medianCCD[k], BLACK);
                    tft_put_pixel(k, 159-schmittCCD[k], BLACK);
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

                avg = (leftEdge+rightEdge)/2;
                drawLine(avg, 0, RED);

                tft_fill_area(38, 50, 80, 20, BLACK);

                drawLine(64-MOVEMENT_SENS, 0, WHITE);
                drawLine(64+MOVEMENT_SENS, 0, WHITE);

                if (avg < 64-MOVEMENT_SENS) {
                    tft_prints(38, 50, "L %d%",
                               clamp(CENTER+ (LEFTMOST-CENTER)*((64-avg)/20.0), RIGHTMOST, LEFTMOST)
                              );
                    servo_control(SERVO1,
                                  clamp(CENTER+ (LEFTMOST-CENTER)*((64-avg)/20.0), RIGHTMOST, LEFTMOST)
                                 );
                } else if (avg > 64+MOVEMENT_SENS) {
                    tft_prints(38, 50, "R %d%",
                               clamp(CENTER- (CENTER-RIGHTMOST)*((avg-64)/20.0), RIGHTMOST, LEFTMOST)
                              );
                    servo_control(SERVO1,
                                  clamp(CENTER- (CENTER-RIGHTMOST)*((avg-64)/20.0), RIGHTMOST, LEFTMOST)
                                 );
                } else {
                    servo_control(SERVO1, CENTER);
                    tft_prints(50, 50, "%d", CENTER);
                }


                for (k = 0; k < 128; k++) { //Add CCD onto Screen
                    tft_put_pixel(k, 159-linear_ccd_buffer1[k], RED);
                    tft_put_pixel(k, 159-schmittCCD[k], GREEN);
                    tft_put_pixel(k, 159-medianCCD[k], WHITE);
                }
                int h;
                for(h = 0; h < 159; h+=20) { //draw label
                    tft_prints(2, h, "%d", 159-h);
                }
            }
            
        }
    }
}
