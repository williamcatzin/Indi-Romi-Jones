/*
	Original Author:		Jeff C. Jensen
    Rewritten By: Joshua Adkins, Neal Jackson
	Revised:	2019-08-12
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// #include <nrf_serial.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "kobukiUART.h"
#include "kobukiUtilities.h"

extern const int *port;
#define NRF_SUCCESS 1
int32_t kobukiReadFeedbackPacket(uint8_t* packetBuffer, uint8_t len){
  
  typedef enum {
    wait_until_AA,
    wait_until_55,
    read_length,
    read_payload,
    read_checksum
  } state_type;

  state_type state = wait_until_AA;

  uint8_t header_buf[2];
  uint8_t byteBuffer;
  int status = 0;
  uint8_t payloadSize = 0;
  uint8_t calcuatedCS;
  size_t paylen;
  size_t aa_count = 0;

  kobukiUARTInit();
  // status = nrf_serial_flush(serial_ref, NRF_SERIAL_MAX_TIMEOUT);
  // if(status != NRF_SUCCESS) {
  //   printf("flush error: %d\n", status);
  //   return status;
  // }
  // status = nrf_serial_rx_drain(serial_ref);
  // if(status != NRF_SUCCESS) {
  //   printf("rx drain error: %d\n", status);
  //   return status;
  // }
  // serialFlush(*serial_ref); ///--------------------------------
  
  int num_checksum_failures = 0;
  if (len <= 4){
    kobukiUARTUnInit();
    return -1;
  }

  while(1){
    // printf("state %d\n", state);
    switch(state){
      case wait_until_AA:
        // status = nrf_serial_read(serial_ref, header_buf, 1, NULL, 100);
          status = read(*port, header_buf, 1);
              
          if(status < 0) {
            printf("UART error reading start of kobuki header: %d\n", header_buf[0]);
            if (aa_count++ < 20) {
              printf("\ttrying again...\n");
              break;
            } else {
              printf("Failed to recieve from robot.\n\tIs robot powered on?\n\tTry unplugging buckler from USB and power cycle robot\n");
            }
            aa_count = 0;
            kobukiUARTUnInit();
            return header_buf[0];
          }
      
          status = read(*port, header_buf + 1, 1);

          if(status < 0) {
          printf("UART error reading last of kobuki header: %d\n", status);
          if (aa_count++ < 20) {
            printf("\ttrying again...\n");
            break;
          } else {
            printf("Failed to recieve from robot.\n\tIs robot powered on?\n\tTry unplugging buckler from USB and power cycle robot\n");
          }
          aa_count = 0;
          kobukiUARTUnInit();
          return status;
          }
            
          
          // if(status != NRF_SUCCESS) {
          //   printf("UART error reading start of kobuki header: %d\n", status);
          // }
          if (header_buf[0]==0xAA && header_buf[1]==0x55) {
            state = read_length;
          } else {
            state = wait_until_AA;
          }
          aa_count = 0;

        break;

      case read_length:
          // status = nrf_serial_read(serial_ref, &payloadSize, sizeof(payloadSize), NULL, 100);
          status = read(*port, (void*)&payloadSize, sizeof(payloadSize));
          if(status < 0) {
            kobukiUARTUnInit();
            return status;
          }

          if(len < payloadSize+3){
              kobukiUARTUnInit();
              return -1;
          }

          state = read_payload;
          // UARTFlush();
          break;

      case read_payload:
          // status = nrf_serial_read(serial_ref, packetBuffer+3, payloadSize+1, &paylen, 100);
          
          // status = read(*port,  packetBuffer + 3, payloadSize + 1);
          status = UARTRead(packetBuffer + 3, payloadSize + 1);
          
          if (status < payloadSize + 1){
            // printf("read %x / %x ",status, payloadSize + 1);
          }
          if(status < 0) {
            kobukiUARTUnInit();
            return status;
          }

          state = read_checksum;

          break;

      case read_checksum:
          memcpy(packetBuffer, header_buf, 2);
          packetBuffer[2] = payloadSize;
          // for (int i = 0; i < payloadSize; i++){
          //   printf("%X", packetBuffer[i]);
          // }

          calcuatedCS = checkSumRead(packetBuffer, payloadSize + 3);
          byteBuffer=(packetBuffer)[payloadSize+3];
          if (calcuatedCS == byteBuffer) {
            num_checksum_failures = 0;
            kobukiUARTUnInit();
            return NRF_SUCCESS;
          } else{
            state = wait_until_AA;
            if (num_checksum_failures == 3) {
              kobukiUARTUnInit();
              return -1500;
            }
            num_checksum_failures++;
          }
          printf("check fails: %d\n", num_checksum_failures);
        break;

      default:
          break;
    }

  }
  kobukiUARTUnInit();
  return status;
}
