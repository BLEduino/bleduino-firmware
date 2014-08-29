/*
BLEduino.h (based NewSoftSerial.h) - 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
http://bleduino.cc
*/


#ifndef BLEduino_h
#define BLEduino_h

#include <inttypes.h>
//#include <Stream.h>

/******************************************************************************
* Definitions
******************************************************************************/

#define _SS_MAX_RX_BUFF 21 // RX buffer size
#define _SS_MAX_PACKET_LENGTH 20 // size of each element in buffer
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

#define BLEDUINO_LED 13

//used for shield-shield compatibility
#define BD11 MOSI
#define BD12 MISO
#define BD13 SCK

/********************************************************************************/
/* User Pipes */

#define CONTROLLER_READ 4
#define CONTROLLER_SEND 2

#define FIRMATA_READ 9
#define FIRMATA_SEND 7

#define UART_READ 12
#define UART_SEND 14

#define NOTIFICATION_SEND 16

#define BRIDGE_ID_READ 21
#define BRIDGE_ID_SEND 22

#define BRIDGE_READ 24
#define BRIDGE_SEND 26

#define VEHICLE_READ 30
#define VEHICLE_SEND 28

#define PIPE_IS_CONNECTED 0

/********************************************************************************/
/* Internal Pipes */

#define PIPE_GAP_DEVICE_NAME_SET 1

#define PIPE_CONTROLLER__BUTTON_ACTION_TX 2
#define PIPE_CONTROLLER__BUTTON_ACTION_TX_MAX_SIZE 5

#define PIPE_CONTROLLER__BUTTON_ACTION_TX_ACK 3
#define PIPE_CONTROLLER__BUTTON_ACTION_TX_ACK_MAX_SIZE 5

#define PIPE_CONTROLLER__BUTTON_ACTION_RX 4
#define PIPE_CONTROLLER__BUTTON_ACTION_RX_MAX_SIZE 5

#define PIPE_CONTROLLER__BUTTON_ACTION_SET 5
#define PIPE_CONTROLLER__BUTTON_ACTION_SET_MAX_SIZE 5

#define PIPE_CONTROLLER__BUTTON_ACTION_RX_ACK_AUTO 6
#define PIPE_CONTROLLER__BUTTON_ACTION_RX_ACK_AUTO_MAX_SIZE 5

#define PIPE_FIRMATA_FIRMATA_COMMAND_TX 7
#define PIPE_FIRMATA_FIRMATA_COMMAND_TX_MAX_SIZE 4

#define PIPE_FIRMATA_FIRMATA_COMMAND_TX_ACK 8
#define PIPE_FIRMATA_FIRMATA_COMMAND_TX_ACK_MAX_SIZE 4

#define PIPE_FIRMATA_FIRMATA_COMMAND_RX 9
#define PIPE_FIRMATA_FIRMATA_COMMAND_RX_MAX_SIZE 4

#define PIPE_FIRMATA_FIRMATA_COMMAND_SET 10
#define PIPE_FIRMATA_FIRMATA_COMMAND_SET_MAX_SIZE 4

#define PIPE_FIRMATA_FIRMATA_COMMAND_RX_ACK_AUTO 11
#define PIPE_FIRMATA_FIRMATA_COMMAND_RX_ACK_AUTO_MAX_SIZE 4

#define PIPE_UART_RX_RX 12
#define PIPE_UART_RX_RX_MAX_SIZE 20

#define PIPE_UART_RX_RX_ACK_AUTO 13
#define PIPE_UART_RX_RX_ACK_AUTO_MAX_SIZE 20

#define PIPE_UART_TX_TX 14
#define PIPE_UART_TX_TX_MAX_SIZE 20

#define PIPE_UART_TX_TX_ACK 15
#define PIPE_UART_TX_TX_ACK_MAX_SIZE 20

#define PIPE_NOTIFICATION_NOTIFICATION_ATTRIBUTES_TX 16
#define PIPE_NOTIFICATION_NOTIFICATION_ATTRIBUTES_TX_MAX_SIZE 20

#define PIPE_NOTIFICATION_NOTIFICATION_ATTRIBUTES_TX_ACK 17
#define PIPE_NOTIFICATION_NOTIFICATION_ATTRIBUTES_TX_ACK_MAX_SIZE 20

#define PIPE_NOTIFICATION_NOTIFICATION_ATTRIBUTES_RX 18
#define PIPE_NOTIFICATION_NOTIFICATION_ATTRIBUTES_RX_MAX_SIZE 20

#define PIPE_NOTIFICATION_NOTIFICATION_ATTRIBUTES_SET 19
#define PIPE_NOTIFICATION_NOTIFICATION_ATTRIBUTES_SET_MAX_SIZE 20

#define PIPE_NOTIFICATION_NOTIFICATION_ATTRIBUTES_RX_ACK_AUTO 20
#define PIPE_NOTIFICATION_NOTIFICATION_ATTRIBUTES_RX_ACK_AUTO_MAX_SIZE 20

#define PIPE_BLE_BRIDGE_DEVICE_ID_RX 21
#define PIPE_BLE_BRIDGE_DEVICE_ID_RX_MAX_SIZE 1

#define PIPE_BLE_BRIDGE_DEVICE_ID_SET 22
#define PIPE_BLE_BRIDGE_DEVICE_ID_SET_MAX_SIZE 1

#define PIPE_BLE_BRIDGE_DEVICE_ID_RX_ACK_AUTO 23
#define PIPE_BLE_BRIDGE_DEVICE_ID_RX_ACK_AUTO_MAX_SIZE 1

#define PIPE_BLE_BRIDGE_BRIDGE_RX_RX 24
#define PIPE_BLE_BRIDGE_BRIDGE_RX_RX_MAX_SIZE 20

#define PIPE_BLE_BRIDGE_BRIDGE_RX_RX_ACK_AUTO 25
#define PIPE_BLE_BRIDGE_BRIDGE_RX_RX_ACK_AUTO_MAX_SIZE 20

#define PIPE_BLE_BRIDGE_BRIDGE_TX_TX 26
#define PIPE_BLE_BRIDGE_BRIDGE_TX_TX_MAX_SIZE 20

#define PIPE_BLE_BRIDGE_BRIDGE_TX_TX_ACK 27
#define PIPE_BLE_BRIDGE_BRIDGE_TX_TX_ACK_MAX_SIZE 20

#define PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_TX 28
#define PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_TX_MAX_SIZE 4

#define PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_TX_ACK 29
#define PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_TX_ACK_MAX_SIZE 4

#define PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_RX 30
#define PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_RX_MAX_SIZE 4

#define PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_SET 31
#define PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_SET_MAX_SIZE 4

#define PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_RX_ACK_AUTO 32
#define PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_RX_ACK_AUTO_MAX_SIZE 4

#define PIPE_TX_POWER_TX_POWER_LEVEL_TX 33
#define PIPE_TX_POWER_TX_POWER_LEVEL_TX_MAX_SIZE 1

/********************************************************************************/
/* Commands */

#define COMMAND_RESET 0

#define COMMAND_DISCONNECT 1

#define COMMAND_IS_CONNECTED 2

#define COMMAND_LED_ON 3

#define COMMAND_LED_OFF 4

#define COMMAND_BAUD_RATE_9600 5

#define COMMAND_BAUD_RATE_14400 6

#define COMMAND_BAUD_RATE_19200 7

#define COMMAND_BAUD_RATE_28800 8

#define COMMAND_BAUD_RATE_31250 9

#define COMMAND_BAUD_RATE_38400 10

#define COMMAND_BAUD_RATE_57600 11

#define COMMAND_SLEEP_TOGGLE 12

#define COMMAND_CHANGE_DEVICE_NAME 13

/********************************************************************************/

typedef struct _DELAY_TABLE
{
  long baud;
  unsigned short rx_delay_centering;
  unsigned short rx_delay_intrabit;
  unsigned short rx_delay_stopbit;
  unsigned short tx_delay;
} DELAY_TABLE;

typedef struct internal_packet {
  uint8_t pipe;
  uint8_t size;
  uint8_t next;
  uint8_t data[20];
} internal_packet;

typedef struct BLEPacket {
  uint8_t pipe;
  uint8_t length;
  uint8_t * data;
} BLEPacket;

class BLEduino// : public Stream
{
private:
  // per object data

  //fancy variables
  uint8_t _receivePin;
  uint8_t _receiveBitMask;
  volatile uint8_t *_receivePortRegister;
  uint8_t _transmitBitMask;
  volatile uint8_t *_transmitPortRegister;

  bool _inverse_logic;

  // static data
  static internal_packet _receive_buffer[_SS_MAX_RX_BUFF];
  static volatile uint8_t _receive_buffer_free_nodes[_SS_MAX_RX_BUFF];
  static volatile uint8_t _free_node_index;
  static volatile uint8_t _receive_buffer_tail;
  static volatile uint8_t _receive_buffer_head;
  static volatile bool _ble_is_connected;
  static volatile bool _ble_name_changed;
  static volatile bool _buffer_overflow;
  static BLEduino *active_object;

  // private (internal) methods
  void recv(uint8_t * recv_address, DELAY_TABLE current_table);
  void read_packet(uint8_t address);
  uint8_t rx_pin_read();
  void tx_pin_write(uint8_t pin_state);
  void setTX(uint8_t transmitPin);
  void setRX(uint8_t receivePin);

  // private static method for timing
  static inline void tunedDelay(uint16_t delay); //don't touch

public:
  // public methods
  BLEduino(uint8_t receivePin = 11, uint8_t transmitPin = 12, bool inverse_logic = false); //Do not change this (unless you're a wizard)
  ~BLEduino();
  void begin(long speed = /*9600 38400 */ 57600); 
  bool listen();
  void end();
  bool isListening() { return this == active_object; }
  bool overflow() { return _buffer_overflow; }
  
  //new functions
  BLEPacket read();
  BLEPacket read(uint8_t pipe); 
  uint8_t available(); 
  uint8_t available(uint8_t pipe); 
  void sendData(uint8_t pipe, uint8_t* data, uint8_t size);
  void sendData(uint8_t pipe, uint8_t data);
  void sendCommand(uint8_t command); 
  void write(uint8_t b); 
  void write_fast(uint8_t b);
  void flush(); 
  void flush(uint8_t pipe);
  BLEPacket peek(); 
  BLEPacket peek(uint8_t pipe); 
  bool isConnected();

  //DEBUG
  internal_packet read_debug();
  void print_all();
  void print_all_list();
  uint8_t return_head();
  uint8_t return_tail();

  // public only for easy access by interrupt handlers
  static inline void handle_interrupt();
};

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round

#endif
