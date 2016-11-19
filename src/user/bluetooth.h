#ifndef _BLUETOOTH_H
#define _BLUETOOTH_H

const int MAXBUFFER = 10; //max buffer size for smartcar bluetooth incoming message
char buffer[MAXBUFFER] = {0}; //stores current input chars
int buffer_index = 0;
bool command_finished;

void bluetooth_init();
void buffer_clear();
void uart_listener(const u8 byte);
void bluetooth_handler();
#endif