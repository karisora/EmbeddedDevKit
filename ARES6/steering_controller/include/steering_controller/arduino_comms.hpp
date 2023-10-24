#ifndef ARDUINO_COMMS__HPP
#define ARDUINO_COMMS__HPP

namespace
{
class ArduinoComms
{
public:
  ArduinoComms(const char* device_name);
  bool connected;
private:
  int open_serial(const char* device_name);
  int fd_;
}; // end of class: ArduinoComms
} // end of namespace

#endif