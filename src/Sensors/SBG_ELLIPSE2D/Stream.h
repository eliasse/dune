#ifndef __STREAM_H__
#define __STREAM_H__

// DUNE headers.
#include <DUNE/DUNE.hpp>
using DUNE_NAMESPACES;

class Stream {

  // Serial port handle.
  Hardware::SerialPort* m_uart;

public:

  Stream() {
    m_uart = new SerialPort("/dev/ttyUSB0", 230400);
  }

  bool available() {
    if (!Poll::poll(*m_uart, 0.001))
      return false;
    return true;
  };

  uint8_t read() {
    uint8_t b;
    int n = m_uart->read(&b, 1);
    if(n==0) std::cout << "ERROR read 0 bytes!" << std::endl;
    std::cout << "stream: read " << n << " bytes. b=" << (int)b << std::endl;
    return b;
  }

  int write(uint8_t* buffer, int len) {return m_uart->write(buffer, len);};

  int write(char c) {return m_uart->write(&c, 1);};

};


#endif //__STREAM_H__
