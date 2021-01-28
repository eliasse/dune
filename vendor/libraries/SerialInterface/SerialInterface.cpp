#include "SerialInterface.h"

#include <iostream>



// Constructor
SerialInterface::SerialInterface() {
  //Do something?
};


void SerialInterface::setup(std::string portName, int baudrate) {
  m_uart = new Hardware::SerialPort(portName, baudrate);
  std::cout << "Serial port created" << std::endl;
  OK = true;
};

void SerialInterface::readData() {
  //std::cout << "Read data called" << std::endl;
  if(!OK) return;

  if(m_uart == NULL) {
    //std::cout << "NULL!" << std::endl;
    return;
  }

  char bfr[1024];
  int rv = 0;
  rv = m_uart->read(bfr, sizeof(bfr));

  //std::cout << "rv = "  << rv << std::endl;

  for(int i=0;i<rv;i++) {
    //printf("received: %d\n", bfr[i]);
    parse_data(bfr[i]);
  }
};

bool SerialInterface::send_data(char* buf, uint8_t len) {
  if(!OK) return false;
  if(m_uart == NULL) return false;
  m_uart->write(buf, len);
  return true;
};

void SerialInterface::end() {
};
