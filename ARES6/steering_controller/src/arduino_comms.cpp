#include "arduino_comms.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <x86_64-linux-gnu/sys/ioctl.h>
#include <string>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <fcntl.h> // "open" is included 
#include <termios.h> 
#include <sstream>
#include <iostream> // std::cout is included

namespace steering_controller
{

ArduinoComms :: ArduinoComms(const char* device_name){
  // Constructor
}

int ArduinoComms :: open_serial(const char * device_name){
  // Begin serial comm
  int fd1;
  // Open port.
  //
  fd1 = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK );

  // If fd1 < 0, its error
  if(fd1 < 0){
    connected = false;
    return -1;
  }

  // Remove flag.
  //
  fcntl(fd1, F_SETFL);
  struct termios conf_tio;
  tcgetattr(fd1,&conf_tio); 

  // Set bound rate
  // Shoule be same as the one in Teensy/main.cpp
  speed_t BAUDRATE = B115200; // baudrate B value
  cfsetispeed(&conf_tio, BAUDRATE); // reading baudrate
  cfsetospeed(&conf_tio, BAUDRATE); // writing baudrate
  
  conf_tio.c_cflag &= ~PARENB; 
  conf_tio.c_cflag &= ~CSTOPB; 
  conf_tio.c_cflag &= ~CSIZE; 
  conf_tio.c_cflag |= CS8;
  /* no hardware flow control */
  conf_tio.c_cflag &= ~CRTSCTS; 
  /* enable receiver, ignore status lines */
  conf_tio.c_cflag |= CREAD | CLOCAL; 
  /* disable input/output flow control, disable restart chars */
  conf_tio.c_iflag &= ~(IXON | IXOFF | IXANY); 
  /* disable canonical input, disable echo,
  disable visually erase chars,
  disable terminal-generated signals */
  conf_tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
  /* disable output processing */
  conf_tio.c_oflag &= ~OPOST; 
      
  
  //non blocking
  conf_tio.c_cc[VMIN]=12; 
  conf_tio.c_cc[VTIME]=0; 
  //store configuration
  tcsetattr(fd1,TCSANOW,&conf_tio); 
  tcflush(fd1, TCIFLUSH); 
  return fd1;
}

} // end of namespace: steering_controller
