
#include "main.h"
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

u32 ticks_img = 0;
u32 ticks_sec_img = 0;
u16 servo_pos = 750;
u8 speed = 20;
u16 speed_indic[3] = {RGB888TO565(0xC72929), RGB888TO565(0xFFC72C), RGB888TO565(0x40CA77)};
const int WINDOWSIZE = 30;
const int MAXBUFFER = 100;

int global_led_on = 0;
int bool_need_clear_buffer = 1;
int bool_command_finish = 0;
int timeSinceLastCommand;
int curTime;
int ccdTime = 0;
int ccd_rate = 50;
int buffer_index = 0;

u8 medianCCD[128] = {0};
u8 schmittCCD[128] = {0};

char buffer[MAXBUFFER] = {0};
char manual_buffer[100];
char curKey = '\0';

void use_motor(long value, long id) {
    if(value<100 && value>0) {
        motor_control((MOTOR_ID)id, 1, value);
    } else {
        uart_tx(COM3, "motor value is out of range\n");
    }
}
void use_servo(long value, long id) {
    if(value<1050&&value>450) {
        servo_control((SERVO_ID)id, value);
    } else {
        uart_tx(COM3, "servo value is out of range\n");
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
void buffer_clear() {
    if (bool_need_clear_buffer) {
        bool_need_clear_buffer = 0;
        buffer_index = 0;
        uart_tx(COM3, "\nBuffer: ");
    }
}
void uart_listener_buffer(const u8 byte) {

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
const int WHITE_THRESHOLD = 130;
void runSchmitt() {
    int k;
    for (k = 0; k < 128; k++) {
        schmittCCD[k] = (medianCCD[k] < WHITE_THRESHOLD)? 1: 158;
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
void init_all() {
    led_init();
    gpio_init();
    ticks_init();
    linear_ccd_init();
    adc_init();
    button_init();
    servo_init(143,10000,0);
    tft_init(1, BLACK, WHITE);
    uart_init(COM3, 115200);
    uart_interrupt_init(COM3, &uart_listener); //com port, function
    uart_tx(COM3, "initialize\n");
    motor_init(143, 10000, 0);
}
/*	if(strcmp("manual",cmd)==0){
							if(curKey=='w'){


							}
						if (curKey != '\0' && get_real_ticks() - curTime >= 200) {
            uart_tx(COM3, "%c released\n", curKey);
            curKey = '\0';
            curTime = get_real_ticks();
        }
			}


						}
						if(strcmp("auto",cmd)==0){


				}*/
int main() {
    init_all();
    while(1) {
        motor_control(0, 1, 5); //id, direction, magnitude
        if (get_real_ticks() - ccdTime >= ccd_rate) { //Update by CCD Rate
            ccdTime = get_real_ticks();
            int k;
            for (k = 0; k < 128; k++) { //Clear CCD Screen
                tft_put_pixel(k, 159-linear_ccd_buffer1[k], BLACK);
                tft_put_pixel(k, 159-medianCCD[k], BLACK);
                tft_put_pixel(k, 159-schmittCCD[k], BLACK);

            }
            linear_ccd_read();
            runMedianFilter();
            runSchmitt();

            for (k = 0; k < 128; k++) { //Add CCD onto Screen
                tft_put_pixel(k, 159-linear_ccd_buffer1[k], RED);
                tft_put_pixel(k, 159-schmittCCD[k], GREEN);

                tft_put_pixel(k, 159-medianCCD[k], WHITE);
            }
            for(h = 0; h < 159; h+=20) {
                tft_prints(2, h, "%d", 159-h);
            }
        }
        bluetooth_handler();
    }
}

//Button listeners
/*
void button1_keydown(void) {led_toggle(LED1);}
void button2_keydown(void) {led_toggle(LED2);}
void button3_keydown(void) {led_toggle(LED3);}
*/
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


//UART listener
void uart3_listener(const u8 byte) {
    u8 ledno = byte - (u8)'1';
    if (ledno >= 3) return;
    led_toggle((LED_ID)ledno);
}
