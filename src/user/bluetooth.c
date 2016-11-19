#include "bluetooth.h"

void bluetooth_init()
{
	uart_init(COM3, 115200);
    uart_interrupt_init(COM3, &uart_listener); //com port, function
    uart_tx(COM3, "initialize\n");
    uart_tx(COM3, ">>> ");
}

void buffer_clear()
{
    buffer_index = 0;
    uart_tx(COM3, "\n >>> ");
}

void uart_listener(const u8 byte)
{
    buffer[buffer_index++] = byte;
    buffer[buffer_index] = '\0';
    uart_tx(COM3, "%c", byte);
    if (byte == '.') {
        bool_command_finish = 1;
    }
    if (byte == 'x') {  //If you make a typo, press x to reset buffer
        buffer_clear();
    }
    if (byte == 'q') {  //Stop Robot
        pid_init(&left_pid, 0, 0, 0);
        pid_init(&right_pid, 0, 0, 0);
        buffer_clear();
    }
}

void bluetooth_handler()
{
    /************************Process Buffer***************************/
    if (command_finish) 
    {
        command_finish = 0;
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
        /************************Commands**********************************/
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
            target_enc_vel = val;
        } else{
            return;
        }
        //uart_tx(COM3, "\n Set " + buffer + " %ld to %d", id, val);
        buffer_clear();
    }
    if (strstr(buffer, "pid")) {  //Check PID values
        uart_tx(COM3, "\n Left  p:%.2f i:%.2f d:%.2f", left_pid.kp, left_pid.ki, left_pid.kd);
        uart_tx(COM3, "\n Right p:%.2f i:%.2f d:%.2f", right_pid.kp, right_pid.ki, right_pid.kd);
        buffer_clear();
    }
    if (strstr(buffer, "derivative")) {
        uart_tx(COM3, "\n Left Prev Time %d, Current Time %d", left_pid.prev.ticks, left_pid.current.ticks);
        uart_tx(COM3, "\n Right Prev Time %d, Current Time %d", right_pid.prev.ticks, right_pid.current.ticks);
        uart_tx(COM3, "\n Left Prev Value %d, Current Value %d", left_pid.prev.value, left_pid.current.value);
        uart_tx(COM3, "\n Right Prev Value %d, Current Value %d", right_pid.prev.value, right_pid.current.value);
        uart_tx(COM3, "\n Derivative Left: %f", left_pid.derivative);
        uart_tx(COM3, "\n Derivative Right: %f", right_pid.derivative);
        buffer_clear();
    }
}