#include "main.h"

u32 ticks_img = 0;
u32 ticks_sec_img = 0;
u16 servo_pos = 750;
u8 speed = 20;
u16 speed_indic[3] = {RGB888TO565(0xC72929), RGB888TO565(0xFFC72C), RGB888TO565(0x40CA77)};

int global_led_on = 0;

const int MAXBUFFER = 100;
char buffer[MAXBUFFER] = {0};
int pointer = 0;

char cmd[10] = "";
char val[10] = "";

int bool_need_clear_buffer = 1;
int bool_command_finish = 0;

char curKey = '\0';

void uart_listener_buffer(const u8 byte) {
    buffer[pointer++] = byte;
    uart_tx(COM3, "BUFFER: %s\n", buffer);

    if (byte == '.') {
        bool_command_finish = 1;
        pointer = 0;
    }
}

int timeSinceLastCommand;
int curTime;

void uart_listener(const u8 byte) {
    curTime = get_real_ticks();
    timeSinceLastCommand = 0;

    if (curKey != byte)
        uart_tx(COM3, "recieved: %c\n", byte);

    curKey = byte;
}

int strequals(const char a[], int aLength, const char b[]) {
    int k;
    for (k = 0; k < aLength; k++) {
        if (a[k] != b[k])
            return 0;
    }
    return 1;
}

const int WINDOWSIZE = 20; //for median
int medianCCD[128] = {0};
int schmittCCD[128] = {0};

float getMedian(const int a[]) {
    int arr[WINDOWSIZE] = {0};
    for (int k = 0; k < WINDOWSIZE; k++) { //copy into temporary array
        arr[k] = a[k];
    }
    int key, j;
    for (int i = 2; i < WINDOWSIZE; i++) { //selection sort
        key = arr[i];
        j = i - 1;
        while (j>0 && arr[j] > key) {
            arr[j+1] = arr[j];
            j--;
        }
        arr[j+1] = key;
    }
    return (WINDOWSIZE%2==1? arr[WINDOWSIZE/2] : (arr[WINDOWSIZE/2-1]+arr[WINDOWSIZE/2])/2);
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

const int WHITE_THRESHOLD = 130;
void runSchmitt() {
    int k;
    for (k = 0; k < 128; k++) {
        schmittCCD[k] = (medianCCD[k] < WHITE_THRESHOLD)? 1: 158;
    }
}

int main() {
    led_init();
    gpio_init();
    ticks_init();
    linear_ccd_init();
    adc_init();
    button_init();

    tft_init(2, BLACK, WHITE);
    uart_init(COM3, 115200);
    uart_interrupt_init(COM3, &uart_listener); //com port, function
    uart_tx(COM3, "initialize\n");

    int ccdTime = 0;

    int h;


    while (1) {

        if (curKey != '\0' && get_real_ticks() - curTime >= 200) {
            uart_tx(COM3, "%c released\n", curKey);
            curKey = '\0';
            curTime = get_real_ticks();
        }

        if (get_real_ticks() - ccdTime >= 50) {
            ccdTime = get_real_ticks();

            int k;
            for (k = 0; k < 128; k++) {
                tft_put_pixel(k, 159-linear_ccd_buffer1[k], BLACK);
                tft_put_pixel(k, 159-medianCCD[k], BLACK);
                tft_put_pixel(k, 159-schmittCCD[k], BLACK);
            }
            linear_ccd_read();
            runMedianFilter();
            runSchmitt();
            for (k = 0; k < 128; k++) {
                //uart_tx(COM3, "%d ", linear_ccd_buffer1[k]);
                tft_put_pixel(k, 159-linear_ccd_buffer1[k], RED);
                tft_put_pixel(k, 159-schmittCCD[k], GREEN);
                tft_put_pixel(k, 159-medianCCD[k], WHITE);
            }
            //uart_tx(COM3, "\n");

            for(h = 0; h < 159; h+=20) {
                tft_prints(2, h, "%d", 159-h);
            }
        }

        if (bool_command_finish) {
            bool_command_finish = 0;
            uart_tx(COM3, "COMPLETE COMMAND: %s\n", buffer);

            int k;
            for (k = 0; buffer[k] != ':'; k++) {
                cmd[k] = buffer[k];
            }
            int j;
            int valPoint = 0;
            for (j = k+1; buffer[j] != '.'; j++) {
                val[valPoint++] = buffer[j];
            }
            uart_tx(COM3, "NAME: %s\n", cmd);
            uart_tx(COM3, "VAL: %s\n", val);

            if (strequals("led", 3, cmd)) {
                if(strequals("1", 1, val)) {
                    led_on(LED1);
                    uart_tx(COM3, "TURNED LED 1 ON\n");
                } else {
                    led_off(LED1);
                    uart_tx(COM3, "TURNED LED 1 OFF\n");
                }
            }

            if (strcmp("led", cmd) == 0) {
                if (strcmp("1", val) == 0) {
                    led_on(LED1);
                    uart_tx(COM3, "TURNED LED 1 ON\n");
                } else {
                    led_off(LED1);
                    uart_tx(COM3, "TURNED LED 1 OFF\n");
                }
            }

            //insert more command checking here

            bool_need_clear_buffer = 1;
        }

        if (bool_need_clear_buffer) {
            bool_need_clear_buffer = 0;
            int k;
            for (k = 0; k < MAXBUFFER; k++) {
                buffer[k] = '\0';
            }
            for (k = 0; k < 10; k++) {
                cmd[k] = '\0';
                val[k] = '\0';
            }
        }
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


int main2() {
    ticks_init();
    led_init();
    gpio_init();
    pneumatic_init();

    button_init();
    set_keydown_listener(BUTTON2, &change_speed);

    tft_init(2, BLACK, WHITE);
    uart_init(COM3, 115200);
    uart_interrupt_init(COM3, &uart_listener); //com port, function
    uart_tx(COM3, "initialize\n");

    tft_prints(0, 0, "tft start");

    tft_init(3, BLACK, RGB888TO565(0xFF0080));
    tft_prints(0, 0, "Hello World!");
    tft_set_text_color(WHITE);
    tft_prints(0, 12, "My name is Pang.");
    tft_prints(0, 24, "I am a single & toxic guy :)");
    tft_prints(0, 60, "Seconds:");
    tft_prints(46, 60, "0");
    tft_prints(0, 72, "Servo:");
    tft_prints(46, 72, "750");
    //tft_set_font(&serifGothic_16ptFontInfo);
    //tft_prints(0, 92, "War Dragon");
    //tft_set_font(&microsoftSansSerif_8ptFontInfo);
    //tft_print_image(0, 120, Logo);

    tft_fill_area(72, 72, 12, 12, RED);

    tft_fill_area(10, 96, 57, 108, WHITE);
    tft_fill_area(12, 98, 55, 106, BLACK);

    uart_init(COM3, 115200);
    uart_tx(COM3, "Test\n");
    uart_interrupt_init(COM3, &uart3_listener);


    servo_init(143, 10000, 0);
    adc_init();
    linear_ccd_init();


    while (1) {

        if (ticks_img != get_real_ticks()) {
            ticks_img = get_real_ticks();
            if (ticks_img % 50 == 0) {
                button_update();

                /*
                if (read_button(BUTTON1) == 0 && servo_pos < 1050) {
                	servo_pos += speed;
                	tft_fill_area(46, 72, 25, 12, BLACK);
                	tft_prints(46, 72, "%d", servo_pos);
                }

                if (read_button(BUTTON3) == 0 && servo_pos > 450) {
                	servo_pos -= speed;
                	tft_fill_area(46, 72, 25, 12, BLACK);
                	tft_prints(46, 72, "%d", servo_pos);
                }
                */
                //}

                if (ticks_img % 1000 == 0) {
                    tft_fill_area(46, 60, 25, 12, BLACK);
                    tft_prints(46, 60, "%d", get_second_ticks());
                }

                if (ticks_img % 10 == 0) {
                    for (u8 i=0; i<128; i++)
                        tft_put_pixel(i, 159 - linear_ccd_buffer1[i], BLACK);
                    linear_ccd_read();
                    for (u8 i=0; i<128; i++)
                        tft_put_pixel(i, 159 - linear_ccd_buffer1[i], WHITE);
                }
            }

            /*servo_control(SERVO1, servo_pos);
            if (read_gpio(GPIOA, GPIO_Pin_11)) {
            	led_on(LED1);
            	pneumatic_control(GPIOC, GPIO_Pin_0, 1);
            } else {
            	led_off(LED1);
            	pneumatic_control(GPIOC, GPIO_Pin_0, 0);
            }*/

        }
    }
}
