/*------------------------------------------------------------------------------------
	Captain scientist interface V0.3: Serial
------------------------------------------------------------------------------------*/

#ifndef SerialInterface_h
#define SerialInterface_h

#include <ISBInterface/ISBInterface.h>
#include <cstring>
#include <iostream>
#include <DUNE/DUNE.hpp>

using DUNE_NAMESPACES;

//----------------------------------------------------------------
class SerialInterface : public ISBInterface {
  bool OK = true;
  // Serial port handle.
  Hardware::SerialPort* m_uart;
protected:
  bool send_data(char* buf, uint8_t len);

public:
  SerialInterface();
  void setup(std::string port, int baudrate);
  void readData();
  void end();
};
//----------------------------------------------------------------
#endif
