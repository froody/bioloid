/**************************************************************************

                Copyright 2008 Martin Andrews <martin.andrew@PLATFORMedia.com>

                This program is free software: you can redistribute it and/or modify
                it under the terms of the GNU General Public License as published by
                the Free Software Foundation, either version 3 of the License, or
                (at your option) any later version.

                This program is distributed in the hope that it will be useful,
                but WITHOUT ANY WARRANTY; without even the implied warranty of
                MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

                You should have received a copy of the GNU General Public License
                along with this program.  If not, see <http://www.gnu.org/licenses/>.

**************************************************************************/

#include <cstdlib>
#include <iostream>
#include <cstring>
#include <math.h>

#include <sstream>
#include <iomanip>

#include <exception>
using namespace std;

#include "bioloid.h"

// for sleep_ms()
#include "util.h"

string output_hex(byte b) {
  stringstream ss;

  // Manipulate the flags.
  ss.unsetf(ios::dec);
  ss.unsetf(ios::showbase);
  ss.setf(ios::uppercase | ios::hex);

  // Put the hex-code into the stringstream object.
  ss << setw(2) << setfill('0') << (int)b;
  return ss.str();
}

string output_hex(vector<byte> v) {
  stringstream ss;
  ss << "{ ";
  for(vector<byte>::iterator p=v.begin(); p != v.end(); p++) {
    ss << output_hex(*p) << " ";
//		if(p != v.end()) { ss << " "; }
  }
  ss << "}";
  return ss.str();
}

// B stands for Bioloid here...

BMessage::BMessage(byte _id, byte _instruction) {
  id=_id;
  instruction=_instruction;
  status=0;
  inhibit_status_return=false;
}

BMessage::~BMessage() {
}

void BMessage::display() {
  cout << "Message "
       << ": id=" << output_hex(id)
       << ", inst=" << output_hex(instruction)
       << ", params=" << output_hex(param)
       << endl;
}

void BMessageQueue::add(BMessage &m, BBusTime _send_at) {
  m.send_at=_send_at;
  boost::mutex::scoped_lock scoped_lock(mutex_for_queue);
  q.push(m);
}

/*
   void BMessageQueue::add_immediate(BMessage &m) {
   BBusTime _send_at=bpt::microsec_clock::local_time();
    add(m, _send_at);
   }
 */

BBusTime BMessageQueue::next_message_time() {
  if(q.empty() ) {
    return BBusTimeEmpty;
  }
  return q.top().send_at;
}

BMessage BMessageQueue::next_message() {
  BMessage m=q.top();
  q.pop();
  return m;
}

BBus::BBus(short _port) {
  abort=false;
// device.resize(255, (BDevice *)NULL);
  device.clear();
  boost::thread t(
    boost::bind(&BBus::StartServer, this, _port)
    );
}

BBus::~BBus() {
  for(map<byte, BDevice *>::iterator dev=device.begin(); dev!=device.end(); ++dev) {
    delete dev->second;
    device.erase(dev);
  }
}

BBusTime BBus::current_bus_time() { // Dependent on time implementation
  return bpt::microsec_clock::local_time();
}

BBusTime BBus::time_plus(BBusTime t, float ms) { // Dependent on time implementation
  bpt::time_duration td = bpt::microseconds( (long)(ms*1000.0) );
  return t+td;
}

// This is a monitor process - that copes with clients disconnecting and reconnecting
void BBus::StartServer(short _port) {
  ba::ip::tcp::acceptor acceptor(
    io_service,
    ba::ip::tcp::endpoint(ba::ip::tcp::v4(), _port)
    );
  while(!abort) {
    cout << "Start Server with port=" << _port << endl;
    bus_socket=new ba::ip::tcp::socket(io_service);
    acceptor.accept(*bus_socket);

    connected=true;
    cout << "Client connected !" << endl;
    boost::thread reader(
      boost::bind(&BBus::TCPtoReceiveQueue, this)
      );
    boost::thread dispatcher(
      boost::bind(&BBus::IncomingMessageDispatcher, this)
      );
    boost::thread writer(
      boost::bind(&BBus::SendQueueToTCP, this)
      );
    reader.join();     // Wait for the reader to finish
    dispatcher.join(); // and the dispatcher
    writer.join();     // and the writer to die too
  }
}

void BBus::TCPtoReceiveQueue() {
  while(connected) {
    // .. Now listen for client commands...
    BMessage message(-1, -1);
    if(read_message_immediate(message) ) {
//			message.display();
      // Add the message to the incoming queue
      receive_queue.add(message, current_bus_time() );
    }

    // don't just fall though here - we're connected...
  }
  cout << "End of : TCPtoReceiveQueue" << endl;
  return;
}

void BBus::IncomingMessageDispatcher() {
  while(connected) {
    bool new_message=false;
    while(!new_message && connected) { // .. Wait on the receive queue mutex thingy
      sleep_ms(1); // Minor sleep (measured in milliseconds)
      if(receive_queue.q.size()>1) {
        cout << "\nLength of Dispatch Queue : " << receive_queue.q.size() << endl;
      }
      if(receive_queue.next_message_time() < current_bus_time() ) {
        new_message=true;
      }
    }

    if(new_message) {
      BMessage message = receive_queue.next_message();
      if(message.id == BROADCASTING_ID) { // This is for SYNC_WRITES
        if( (message.instruction == INST_SYNC_WRITE) || (message.instruction == INST_SYNC_REG_WRITE) ) {
          cout << "Dispatching Sync Messages" << endl;
          byte instruction_common=(message.instruction == INST_SYNC_WRITE) ? INST_WRITE : INST_REG_WRITE;
          if(message.param.size()<2) {
            cerr << "SYNC_WRITE parameter length problem" << endl;
          }
          else {
            byte location=message.param[0];
            byte length=message.param[1]; // This is length of each data (not counting id)
            int pos=2;
            while(pos<message.param.size() ) {
              byte ind_id=message.param[pos];
              if(device.count(ind_id)==0) {
                cerr << "Individual Message for device # " << output_hex(ind_id) << ", which isn't connected" << endl;
              }
              else {
                BMessage message_individual(ind_id, instruction_common);
                message_individual.param.push_back(location);
                for(int i=1; i<=length; i++) {
                  message_individual.param.push_back(message.param[pos+i]);
                }
                cout << " Dispatching Sync'd - ";
                message_individual.inhibit_status_return=true;
                message_individual.display();
                device[ind_id]->handle_message(message_individual, this);
              }
              pos += (1+length);  // id is not counted in length
            }
          }
        }
        else { // Not a SYNC write message - broadcast this to all attached devices... (likely to be ACTION)
          for(map<byte, BDevice *>::iterator dev=device.begin(); dev!=device.end(); dev++) {
            (*dev).second->handle_message(message, this); // Need a broadcast ID to stop the status return, so don't individualize
          }
        }
      }
      else if(device.count(message.id)==0) {
        cerr << "Message for device # " << output_hex(message.id) << ", which isn't connected" << endl;
      }
      else {
        cout << "Dispatching single-target - ";
        message.display();
        device[message.id]->handle_message(message, this);
      }
    }
  }
  cout << "End of : IncomingMessageDispatcher" << endl;
  return;
}

void BBus::SendQueueToTCP() {
  // Detect if connected
  while(connected) {
    // .. Now wait for something to arrive in the MessageSend queue
//		sleep_ms(1000);
    sleep_ms(1);
    if(send_queue.next_message_time() < current_bus_time() ) {
      if(send_queue.q.size()>1) {
        cout << "Length of Send Queue : " << send_queue.q.size() << endl;
      }
      BMessage message=send_queue.next_message();
      send_message_immediate(message);
    }
  }
  cout << "End of : SendQueueToTCP" << endl;
  return;
}

void BBus::fill_buffer_exactly(vector<byte> &buf, ba::error_code &e) {
  size_t len = 0;
  size_t n=buf.size();

  do {
    len += bus_socket->read_some(ba::buffer(&buf[len], n-len), e);
  } while( (len < n) && (!e) );

  if(len == n) {
//		cout << "Read " << n << " bytes" << endl;
    return;
  }
  if(e != ba::error::eof) { // Ignore EOFs for this complaint...
//		throw(exception(e.message().c_str()));
    cerr << "read_n_bytes(" << n << ") failed : " << e.message().c_str() << endl;
  }
  return;
}

bool BBus::read_message_immediate(BMessage &m) {
  ba::error_code e;
  vector<byte> header(5);
  fill_buffer_exactly(header, e);
  if(e) { // an error of some kind
    if(e == ba::error::eof) {
      cerr << "Got an eof - Client has disconnected" << endl;
      connected=false;
      return false;
    }
  }

  if( (header[0]==0xFF) && (header[1]==0xFF) ) { // Header is Ok
    m.id=header[2];
    byte len=header[3];
    m.instruction=header[4];

//		cout << "Header : " << output_hex(header) << endl;

    m.param.clear();
    m.param.resize( (int)len-1); // This is 1 longer than the parameters (also read crc)
    fill_buffer_exactly(m.param, e);

//		cout << "Params : " << output_hex(m.param) << endl;

    int crc_calc=m.id + len + m.instruction;
    for(vector<byte>::iterator p=m.param.begin(); p != m.param.end(); p++) {
      crc_calc += *p;
    }
//		byte crc_declared=m.param.back();
    m.param.pop_back(); // So that parameter length is correct

    if( (crc_calc & 0xFF) == 0xFF) {
//			cout << "CRC Match!" << endl;
    }
    else {
      cerr << "CRC Failure! : " << output_hex( (byte)(crc_calc & 0xFF) ) << endl;
      m.status |= STATUS_ERROR_CHECKSUM;
      return true; // Successful read, but return error in message.status
    }
  }
  else { // Bad message start
    cerr << "Bad Message start!" << endl;
    m.status != STATUS_ERROR_CHECKSUM;
    return true; // Successful read, but return error in message.status
  }
  return true;
}

void BBus::send_whole_buffer(vector<byte> &buf, ba::error_code &e) {
  size_t len = 0;
  size_t n=buf.size();

  do {
    len += bus_socket->write_some(ba::buffer(&buf[len], n-len), e);
  } while( (len < n) && (!e) );

  if(len == n) {
//    cout << "Sent " << n << " bytes" << endl;
    return;
  }
  if(e != ba::error::eof) { // Ignore EOFs for this complaint...
//		throw(exception(e.message().c_str()));
    cerr << "send_n_bytes(" << n << ") failed : " << e.message().c_str() << endl;
  }
  return;
}

void BBus::send_message_immediate(BMessage &m) {
  ba::error_code e;
  vector<byte> buffer(5);
  buffer[0]=0xFF;
  buffer[1]=0xFF;
  buffer[2]=m.id;
  buffer[3]=m.param.size()+2;
  buffer[4]=m.instruction;

  int crc_calc=buffer[2]+buffer[3]+buffer[4];

  for(vector<byte>::iterator p=m.param.begin(); p != m.param.end(); p++) {
    buffer.push_back(*p);
    crc_calc += *p;
  }

  crc_calc = ~crc_calc & 0xFF;
  buffer.push_back(crc_calc);

//  cout << "Sending back : " << output_hex(buffer) << endl;

//	bus_socket->write_some(ba::buffer(&buffer, buffer.size()), e);
  send_whole_buffer(buffer, e);
//	bus_socket->flush();
}

bool BBus::connect_device(BDevice *dev_ptr) {
  byte _id=dev_ptr->id;
  if(device.count(_id)>0) {
    cerr << "Bioloid Device " << output_hex(_id) << " already exists on bus - ignoring" << endl;
    return false;
  }
  // else ...
  device[_id]=dev_ptr;
  return true; // success
}

