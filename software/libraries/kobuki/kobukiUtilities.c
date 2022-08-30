#include "kobukiUtilities.h"
#include <stdint.h>
#include <stdio.h>

// #include "nrf_drv_clock.h"
// #include "nrf_uarte.h"
// #include "nrf_serial.h"
// #include "app_timer.h"

// #include "buckler.h"

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/ioctl.h>

#include "kobukiSensorTypes.h"

#include "kobukiActuator.h"

// NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
//                       BUCKLER_UART_RX, BUCKLER_UART_TX,
//                       0, 0,
//                       NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
//                       NRF_UART_BAUDRATE_115200,
//                       UART_DEFAULT_CONFIG_IRQ_PRIORITY);

// #define SERIAL_FIFO_TX_SIZE 512
// #define SERIAL_FIFO_RX_SIZE 512

// NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);


// #define SERIAL_BUFF_TX_SIZE 1
// #define SERIAL_BUFF_RX_SIZE 1

// NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

// NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_DMA,
//                       &serial_queues, &serial_buffs, NULL, NULL);


// NRF_SERIAL_UART_DEF(serial_uart, 0);

// const nrf_serial_t * serial_ref = &serial_uart;

  int serial_port;
  const int *port = &serial_port;
int kobukiUARTInit() {
  //printf("-----------Initializing UART-----------\n");
  serial_port = open("/dev/ttyAMA0", O_RDWR);
  // printf("(in port %d)", serial_port);
  // Check for errors
  if (serial_port < 0) {
      printf("Error %i from open: %s\n", errno, strerror(errno));
      return -1;
  }

  struct termios UART;
        if (tcgetattr(serial_port, &UART) != 0) {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        }

        UART.c_cflag &= ~PARENB; //disable parity
        UART.c_cflag &= ~CSTOPB; //one stop bit
        UART.c_cflag |= CS8; //8 bits
        UART.c_cflag &= ~CRTSCTS; //disable flow controll
        UART.c_cflag |= CREAD | CLOCAL;
        UART.c_lflag &= ~ICANON; //disable cannonical

        UART.c_lflag &= ~ECHO; // Disable echo
        UART.c_lflag &= ~ECHOE; // Disable erasure
        UART.c_lflag &= ~ECHONL; // Disable new-line echo

        UART.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        UART.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        UART.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
        UART.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        UART.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        UART.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        UART.c_cc[VMIN] = 0; 
        cfsetispeed(&UART,B115200);
        cfsetospeed(&UART,B115200);

        // Save tty settings, also checking for error
        if (tcsetattr(serial_port, TCSANOW, &UART) != 0) {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        }
        UARTFlush();
        return 1;
}

void UARTFlush(){
  tcflush(serial_port, TCIFLUSH);
        int bytes_available;
        int retval = ioctl(serial_port, FIONREAD, &bytes_available);
        if (retval < 0) {
          perror("FIONREAD ioctl failed: ");
          exit(7);
        }
        // printf("UART initialized");
        
}

int UARTRead(uint8_t * buffer, uint8_t len){

  int byte =0;
  for (int i = 0; i < len; i++){
    byte = read(serial_port, buffer + i, 1);
    if (byte != 1){
      i++;
    }
  }
  return len;
}

int kobukiUARTUnInit() {
  //printf("----------Unitializing UART------------\n");
  if (close(serial_port) != 0){
    printf("couldn't close UART\n");
    printf("srial port is: %d, *port is %d", serial_port, *port);
    return -1;
  }
  return 1;
}

// int kobukiInit() {
//   uint32_t err_code;

//   err_code = nrf_drv_clock_init();
//   if (err_code != NRF_ERROR_MODULE_ALREADY_INITIALIZED){
//     APP_ERROR_CHECK(err_code);
//   }

//   nrf_drv_clock_lfclk_request(NULL);
//   err_code = app_timer_init();
//   if (err_code != NRF_ERROR_MODULE_ALREADY_INITIALIZED){
//     APP_ERROR_CHECK(err_code);
//   }

//   return err_code;
// }

uint8_t checkSumRead(uint8_t * buffer, int length){
	int i;
    uint8_t cs = 0x00;
    //printf("\n l = %d \n",length);
    for(i = 2; i < length; i++ ){
    //	printf("%d ",buffer[i]);
    	 cs ^= buffer[i];
    }
   // printf("\n last[%d]=%d cs=%d",i,buffer[i],cs);
    return cs;
}
uint8_t checkSum(uint8_t * buffer, int length){
	int i;
    uint8_t cs = 0x00;
    //printf("\n l = %d \n",length);
    for(i = 2; i < length; i++ ){
    	//printf("%d ",buffer[i]);
    	 cs ^= buffer[i];
    }
    //printf("\n last[%d]=%d cs=%d",i,buffer[i],cs);
    return cs;
}

// // checks for the state change of a button press on any of the Kobuki buttons
// bool is_button_pressed(KobukiSensors_t* sensors) {
//   // save previous states of buttons
//   static bool previous_B0 = false;
//   static bool previous_B1 = false;
//   static bool previous_B2 = false;

//   bool result = false;

//   // check B0
//   bool current_B0 = sensors->buttons.B0;
//   if (current_B0 && previous_B0 != current_B0) {
//     result = true;
//   }
//   previous_B0 = current_B0;

//   // check B1
//   bool current_B1 = sensors->buttons.B1;
//   if (current_B1 && previous_B1 != current_B1) {
//     result = true;
//   }
//   previous_B1 = current_B1;

//   // check B2
//   bool current_B2 = sensors->buttons.B2;
//   if (current_B2 && previous_B2 != current_B2) {
//     result = true;
//   }
//   previous_B2 = current_B2;

//   return result;
// }

