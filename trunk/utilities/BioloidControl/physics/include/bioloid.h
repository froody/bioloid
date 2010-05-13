#ifndef __BIOLOID_H__
#define __BIOLOID_H__

#include <vector>
#include <queue>
#include <string>

// #ifdef __cplusplus
// extern "C" {
// #endif

// This is for exceptions
// #include <boost/system/error_code.hpp>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

// cout << "Boost version =" << BOOST_VERSION << endl;
#define ASIO_IN_BOOST 103500

#if BOOST_VERSION < ASIO_IN_BOOST
// #include "../../../boost/asio-1.0.0/include/asio.hpp"
 #include <asio.hpp>
namespace ba=asio;
# else
 #include <boost/asio.hpp>
namespace ba=boost::asio;
#endif

#include <boost/date_time/posix_time/posix_time_types.hpp>
namespace bpt=boost::posix_time;
typedef bpt::ptime BBusTime;

// The BBusTimeEmpty should be greater than any possible current time
#define BBusTimeEmpty bpt::pos_infin

#include "constants.h"

// #if SIZEOF_CHAR == 1
// typedef uint8 byte;
typedef unsigned char byte;
// #else
// #error "expecting sizeof(char) == 1"
// #endif

string output_hex(byte b);
string output_hex(vector<byte> v);

class BMessage {
  friend class BMessageSorter; // for Compare()
  friend class BMessageQueue;  // for send_at accessibility

public:
  byte id;
  byte instruction;
  vector<byte> param;

  byte status; // Records errors like CRC and INSTRUCTION

  bool inhibit_status_return;

  BMessage(byte _id, byte _instruction);
  ~BMessage();
  void display();

private:
  BBusTime send_at;
};

class BMessageSorter {
public:
  int operator()(BMessage &a, BMessage &b) {
    return(a.send_at < b.send_at);
  }

};

class BMessageQueue {
public:
  priority_queue<BMessage, vector<BMessage>, BMessageSorter> q;

// mutex safeguards TCP server queue additions from ODE removals
  boost::mutex mutex_for_queue;

  void add(BMessage &m, BBusTime send_at);

//  void add_immediate(BMessage &m);

  BBusTime next_message_time();
  BMessage next_message();

};

class BDevice;

class BBus {
public:
  BMessageQueue send_queue;
  BMessageQueue receive_queue;

  bool abort;

  BBus(short port);
  ~BBus();

  bool connect_device(BDevice *dev_ptr); // true for success
  BBusTime current_bus_time();
  BBusTime time_plus(BBusTime t, float ms);

private:
  ba::io_service io_service;
  ba::ip::tcp::socket *bus_socket;

  bool connected;

  void StartServer(short _port);

  void TCPtoReceiveQueue();
  void IncomingMessageDispatcher();
  void SendQueueToTCP();

  void fill_buffer_exactly(vector<byte> &buf, ba::error_code &e);
  void send_whole_buffer(vector<byte> &buf, ba::error_code &e);

  bool read_message_immediate(BMessage &m); // true for success : message read (maybe a bad message - see m.status)
  void send_message_immediate(BMessage &m);

  map<byte, BDevice *> device;
};

class BDevice {
public:
  byte id;

  BDevice(byte _id);
  ~BDevice();

  bool handle_message(BMessage &m, BBus *bus); // true for success
  void reset_base_device();

  virtual void reset()=0;
  virtual void about_to_read_memory(int addr_lo, int addr_hi)=0;
  virtual void just_written_memory(int addr_lo, int addr_hi)=0;

//	private:

protected:
  byte memory[255];
  queue<BMessage> queued_messages;
};

#include "servo.h"

class BDeviceAX12 : public BDevice {
public:
  BDeviceAX12(byte _id, Servo *_servo);
//	 void reset_to_factory_settings();

  void about_to_read_memory(int addr_lo, int addr_hi);
  void just_written_memory(int addr_lo, int addr_hi);

//  void setServo(Servo *_servo);
  void reset();

private:
  Servo *servo;
};

#include "sensor.h"

class BDeviceIMUideal : public BDevice {
public:
  BDeviceIMUideal(byte _id, Sensor *_sensor);

  void about_to_read_memory(int addr_lo, int addr_hi);
  void just_written_memory(int addr_lo, int addr_hi);

  void reset();

private:
  Sensor *sensor;
};


/* closing bracket for extern "C" */
// #ifdef __cplusplus
// }
// #endif

#endif

