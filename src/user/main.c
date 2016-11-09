#include "main.h"
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

const int IS_SHOOTER_ROBOT = 0; //to decide which while loop to use

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

int encoderArr[4][4] = {
    {0, -1, 1, 0},
    {1, 0, 0, -1},
    {-1, 0, 0, 1},
    {0, 1, -1, 0}
};

//////carrier robot (smartcar)
//bluetooth
const int MAXBUFFER = 100; //max buffer size for smartcar bluetooth incoming message
char buffer[MAXBUFFER] = {0}; //stores current input chras
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
int clamp(int val, int min, int max) {
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
    if (bool_need_clear_buffer) {
        bool_need_clear_buffer = 0;
        buffer_index = 0;
        uart_tx(COM3, "\nBuffer: ");
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
    /*if (curKey != byte){
        uart_tx(COM3, "received: %c\n", byte);
    }
    curKey = byte;*/
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
        }
        bool_need_clear_buffer = 1;
    }
    /*//TODO: Consider adding a manual command so that letters do not overlap? Test if current program will work on STM32
    int i;
    for (i = 0; buffer[i] != '\0'; ++i) //to account for the rare simulateneous inputs
    {
    	if (buffer[i] == 'w'){ //considering switch statements for readibility
    		uart_tx(COM3, "w "); //up arrow 0x0E
    	}
    	else if (buffer[i] == 'a'){//How to account for the a in 'pneumatic'
    		uart_tx(COM3, "a "); //left arrow 0x0B
    	}
    	else if (buffer[i] == 's'){ //How to account for the s when typing in the command 'servo'
    		uart_tx(COM3, "s "); //down arrow 0x0C
    	}
    	else if (buffer[i] == 'd'){
    		uart_tx(COM3, "d "); //right arrow 0x07
    	}
    }
    if(buffer[i] == '\0' ){ //if the first element of buffer is \0 or the we reached the end of the wasd loop
    	bool_need_clear_buffer = 1; //TODO:Set a time interval where the command is not nullified
    }*/
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

void init_encoder_left() {
    //tim4
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = 65535; // Maximal
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // TIM_EncoderMode_TI1: Counter counts on TI1FP1 edge depending on TI2FP2 level.
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_Cmd(TIM4, ENABLE);

}
void init_encoder_right() {
    //tim3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


    GPIO_PinRemapConfig( GPIO_FullRemap_TIM3, ENABLE );

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = 65535; // Maximal
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // TIM_EncoderMode_TI1: Counter counts on TI1FP1 edge depending on TI2FP2 level.
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_Cmd(TIM3, ENABLE);

}




/*****************************unused************************/
void RCC_Configuration(void) {
    // clock for GPIO and AFIO (for ReMap)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);


    // clock for TIM3, 4
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
}


void GPIO_Configuration(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // PC.06 TIM3_CH1, PC.07 TIM3_CH2 left tim3
    // PB6 TIM4_CH1, PB7 TIM4_CH2 right tim4

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    GPIO_PinRemapConfig( GPIO_FullRemap_TIM3, ENABLE );        // Map TIM3 to GPIOC
}



void TIM3_Configuration(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = 65535; // Maximal
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // TIM_EncoderMode_TI1: Counter counts on TI1FP1 edge depending on TI2FP2 level.
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_Cmd(TIM3, ENABLE);
}

void TIM4_Configuration(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = 65535; // Maximal
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // TIM_EncoderMode_TI1: Counter counts on TI1FP1 edge depending on TI2FP2 level.
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_Cmd(TIM4, ENABLE);
}

/*****************************************************/

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
    motor_init(143, 10000, 0);


    if (IS_SHOOTER_ROBOT == 1) {

        init_encoder_left();
        init_encoder_right();

        while (1) {
            tft_clear();
            tft_prints(10, 10, "left: %d", TIM4->CNT);
            tft_prints(10, 20, "right: %d", TIM3->CNT);
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
            bluetooth_handler();
        }

    }

}
