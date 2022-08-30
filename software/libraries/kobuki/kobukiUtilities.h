
#ifndef KOBUKIUTILITIES
#define KOBUKIUTILITIES

#include <stdint.h>

#include "kobukiSensorTypes.h"

int kobukiInit();
int kobukiUARTInit();
int kobukiUARTUnInit();
void UARTFlush();
int UARTRead(uint8_t * buffer, uint8_t len);
// int read_str(int serial, void * dst, int size);
uint8_t checkSumRead(uint8_t * buffer, int length);
uint8_t checkSum(uint8_t * buffer, int length);

// checks for the state change of a button press on any of the Kobuki buttons
bool is_button_pressed(KobukiSensors_t* sensors);

#endif
