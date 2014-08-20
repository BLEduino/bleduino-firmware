/*
BLEduino.cpp (based on NewSoftSerial.cpp) - 

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
http://arduiniana.org.
*/

//We're running the BLEduino at 57600 (very fast) baud, so the debug options in SoftwareSerial
//would have affected the timings to the point of failure. 

// 
// Includes
// 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <BLEduino.h>
//
// Lookup table
//

#if F_CPU == 16000000

/* 57600 (will break for most applications)
static DELAY_TABLE table_a = {57600, 2, 37, 60, 33};
static DELAY_TABLE table_b = {57600, 0, 37, 50, 33};
static DELAY_TABLE table_c = {57600, 0, 37, 60, 33};
static DELAY_TABLE table_d = {57600, 0, 36, 0, 33};
*/

/* 31250  */
static DELAY_TABLE table_a = {31250,   0, 70, 130, 68};
static DELAY_TABLE table_b = {31250,   0, 70, 130, 68};
static DELAY_TABLE table_c = {31250,   0, 70, 135, 68};
static DELAY_TABLE table_d = {31250,   0, 70,  0,  68};

const int XMIT_START_ADJUSTMENT = 5;

#else

#error This version of BLEduino supports only 20MHz processors

#endif

//
// Statics
//
BLEduino *BLEduino::active_object = 0;
internal_packet BLEduino::_receive_buffer[_SS_MAX_RX_BUFF];
volatile uint8_t BLEduino::_receive_buffer_free_nodes[_SS_MAX_RX_BUFF];
volatile uint8_t BLEduino::_free_node_index = 0;
volatile uint8_t BLEduino::_receive_buffer_tail = 0;
volatile uint8_t BLEduino::_receive_buffer_head = 0;
volatile bool BLEduino::_ble_is_connected = false;
volatile bool BLEduino::_ble_name_changed = false;
volatile bool BLEduino::_buffer_overflow = false;

volatile bool data_received = false;

BLEPacket EMPTY_BLEPACKET = {0, 0, 0};

//
// Private methods
//

/* static */ 
inline void BLEduino::tunedDelay(uint16_t delay) { 
  uint8_t tmp=0;

  asm volatile("sbiw    %0, 0x01 \n\t"
    "ldi %1, 0xFF \n\t"
    "cpi %A0, 0xFF \n\t"
    "cpc %B0, %1 \n\t"
    "brne .-10 \n\t"
    : "+r" (delay), "+a" (tmp)
    : "0" (delay)
    );
}

// This function sets the current object as the "listening"
// one and returns true if it replaces another 
////// This is a feature of SoftwareSerial that this library doens't need.
    // This will probably be removed in the future. Probably.
bool BLEduino::listen()
{
  if (active_object != this)
  {
    _buffer_overflow = false;
    uint8_t oldSREG = SREG;
    cli();
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;
    SREG = oldSREG;
    return true;
  }



  return false;
}

bool BLEduino::isConnected(){
  return _ble_is_connected;
}

//
// The receive routine called by the interrupt handler
//
void BLEduino::recv(uint8_t * recv_address, DELAY_TABLE current_table)
{

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Preserve the registers that the compiler misses
// (courtesy of Arduino forum user *etracer*)
  asm volatile(
    "push r18 \n\t"
    "push r19 \n\t"
    "push r20 \n\t"
    "push r21 \n\t"
    "push r22 \n\t"
    "push r23 \n\t"
    "push r26 \n\t"
    "push r27 \n\t"
    ::);
#endif  

  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (!rx_pin_read())
  {
    // Wait approximately 1/2 of a bit width to "center" the sample
    tunedDelay(current_table.rx_delay_centering ); //tunedDelay(_rx_delay_centering);

    // Read each of the 8 bits
    for (uint8_t i=0x1; i; i <<= 1)
    {
      tunedDelay(current_table.rx_delay_intrabit); //tunedDelay(_rx_delay_intrabit);

      uint8_t noti = ~i; //FAT (0000_0100)
      if (rx_pin_read())
        d |= i;
      else // else clause added to ensure function timing is ~balanced
        d &= noti;
    }

    // skip the stop bit
    tunedDelay(current_table.rx_delay_stopbit); //tunedDelay(_rx_delay_stopbit);

    *recv_address = d; //d
    data_received = true;

  }

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Restore the registers that the compiler misses
  asm volatile(
    "pop r27 \n\t"
    "pop r26 \n\t"
    "pop r23 \n\t"
    "pop r22 \n\t"
    "pop r21 \n\t"
    "pop r20 \n\t"
    "pop r19 \n\t"
    "pop r18 \n\t"
    ::);
#endif
}

void BLEduino::tx_pin_write(uint8_t pin_state)
{
  if (pin_state == LOW)
    *_transmitPortRegister &= ~_transmitBitMask;
  else
    *_transmitPortRegister |= _transmitBitMask;
}

uint8_t BLEduino::rx_pin_read()
{
  return *_receivePortRegister & _receiveBitMask;
}

//
// Interrupt handling
//

/* static */
inline void BLEduino::handle_interrupt()
{

  //Sets Pin 3 High.  
  //Use this for debugging
  //Use with caution, timings will be affected, so if your packets are too big, you'll lose data.
  //PORTD = PORTD | B00000001; 

  // if -> no more free nodes
  if(_free_node_index == _SS_MAX_RX_BUFF){
    // read incoming packet into current head
    active_object->read_packet(_receive_buffer_head);

    if(data_received && (_receive_buffer[_receive_buffer_head].pipe != 0) && !_ble_name_changed){
      
      // tail = head
      _receive_buffer_tail = _receive_buffer_head;

      // head = head + 1
      _receive_buffer_head = _receive_buffer[_receive_buffer_head].next;

      _buffer_overflow = true;
    }

    //buffer full, but received is_connected pipe or OTA name change.
    //write to head, check value, then free head
    if(data_received && ((_receive_buffer[_receive_buffer_head].pipe == PIPE_IS_CONNECTED) || _ble_name_changed)){

      _receive_buffer_free_nodes[--_free_node_index] = _receive_buffer_head;

      _receive_buffer_head = _receive_buffer[_receive_buffer_head].next;
      _receive_buffer[_receive_buffer_tail].next = _receive_buffer_head;
      _free_node_index--;

    }
  }

  else{
    //read incoming packet into current free node
    active_object->read_packet(_receive_buffer_free_nodes[_free_node_index]);

    if(data_received && (_receive_buffer[_receive_buffer_free_nodes[_free_node_index]].pipe) && !_ble_name_changed ){

      //update old tails pointer (point to new packet)
      _receive_buffer[_receive_buffer_tail].next = _receive_buffer_free_nodes[_free_node_index];

      //update tail
      _receive_buffer_tail = _receive_buffer_free_nodes[_free_node_index++];

      //update new tails pointer (point to head)
      _receive_buffer[_receive_buffer_tail].next = _receive_buffer_head;

    }
  }

  data_received = false;
  _ble_name_changed = false;

  //Sets Pin 3 Low 
  //Use this for debugging.
  //PORTD = PORTD & B11111110; 

}

void inline BLEduino::read_packet(uint8_t address){

  //read pipe
  active_object->recv(&_receive_buffer[address].pipe, table_a);

  //read size
  active_object->recv(&_receive_buffer[address].size, table_b);

  if(_receive_buffer[address].size > _SS_MAX_PACKET_LENGTH || _receive_buffer[address].size == 0){
    data_received = false;
    return;
  }
  
  //read data
  for(uint8_t i = 0; i < _receive_buffer[address].size; i++){

    if(i == _receive_buffer[address].size && i > 0){
      active_object->recv(&_receive_buffer[address].data[i], table_d);
    }

    else{
      active_object->recv(&_receive_buffer[address].data[i], table_c);
    }

  }

  if( data_received && (_receive_buffer[address].pipe == PIPE_IS_CONNECTED)){
    _ble_is_connected = _receive_buffer[address].data[0];
  }

  
  if(data_received && (_receive_buffer[address].pipe == BRIDGE_READ) && (_receive_buffer[address].data[0] == 0xFF)){
    //_ble_name_changed = true;
    //Serial.println("Received name change request");
    //Serial.print("_receive_buffer[address].size: "); Serial.println(_receive_buffer[address].size);
    active_object->write(_receive_buffer[address].size + 1);
    active_object->write(0xC0);
    active_object->write(0xCD);

    for(uint8_t i = 1; i < _receive_buffer[address].size; i++){
      active_object->write(_receive_buffer[address].data[i]);
      //Serial.print("character: ["); Serial.print(i); Serial.print("]: "); Serial.println(_receive_buffer[address].data[i]);
    }
  }
  

  //Serial.println("Entered read_packet");
    
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
  BLEduino::handle_interrupt();
}
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect)
{
  BLEduino::handle_interrupt();
}
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect)
{
  BLEduino::handle_interrupt();
}
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect)
{
  BLEduino::handle_interrupt();
}
#endif

//
// Constructor
//
BLEduino::BLEduino(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /* = false */) : 
  _inverse_logic(inverse_logic)
{
  setTX(transmitPin);
  setRX(receivePin);
}

//
// Destructor
//
BLEduino::~BLEduino()
{
  end();
}

void BLEduino::setTX(uint8_t tx)
{
  pinMode(tx, OUTPUT);
  digitalWrite(tx, HIGH);
  _transmitBitMask = digitalPinToBitMask(tx);
  uint8_t port = digitalPinToPort(tx);
  _transmitPortRegister = portOutputRegister(port);
}

void BLEduino::setRX(uint8_t rx)
{
  pinMode(rx, INPUT);
  if (!_inverse_logic)
    digitalWrite(rx, HIGH);  // pullup for normal logic!
  _receivePin = rx;
  _receiveBitMask = digitalPinToBitMask(rx);
  uint8_t port = digitalPinToPort(rx);
  _receivePortRegister = portInputRegister(port);
}

//
// Public methods
//

void BLEduino::begin(long speed)
{

  //DDRD = DDRD | B00000001; //Set pin 3 as output.  For debugging with an oscilloscope.

  //Initialize free_nodes list
  for(uint8_t i = 0; i < _SS_MAX_RX_BUFF; i++){
    _receive_buffer_free_nodes[i] = i;
  }

  // Set up RX interrupts, but only if we have a valid RX baud rate
  if (table_a.baud)
  {
    if (digitalPinToPCICR(_receivePin))
    {
      *digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
      *digitalPinToPCMSK(_receivePin) |= _BV(digitalPinToPCMSKbit(_receivePin));
    }
    tunedDelay(table_a.tx_delay); // if we were low this establishes the end
  }

  listen();

  active_object->write_fast(0x03);
  active_object->write_fast(0xC0);
  active_object->write_fast(0xCB);
  active_object->write_fast(0x05);

}

void BLEduino::end()
{
  if (digitalPinToPCMSK(_receivePin))
    *digitalPinToPCMSK(_receivePin) &= ~_BV(digitalPinToPCMSKbit(_receivePin));
}


// Read data from buffer
BLEPacket BLEduino::read()
{

  //Serial.print("table_a.rx_delay_centering = "); Serial.println(table_a.rx_delay_centering);
  //Serial.print("table_b.rx_delay_intrabit = "); Serial.println(table_a.rx_delay_intrabit);
  //Serial.print("table_c.rx_delay_stopbit = "); Serial.println(table_a.rx_delay_stopbit);

  // Empty buffer?
  if (_free_node_index == 0)
    return EMPTY_BLEPACKET;

  // Read from "head"
  BLEPacket d;
  //BLEPacket d = &p;
  d.pipe    = _receive_buffer[_receive_buffer_head].pipe;
  d.length  = _receive_buffer[_receive_buffer_head].size;
  d.data    = _receive_buffer[_receive_buffer_head].data;

  //make note of new free node
  _receive_buffer_free_nodes[--_free_node_index] = _receive_buffer_head;
  _buffer_overflow = false;

  //move pointers
  _receive_buffer_head = _receive_buffer[_receive_buffer_head].next;
  _receive_buffer[_receive_buffer_tail].next = _receive_buffer_head;

  return d;
}
////////////////////////////////////////////////////////////
//DEBUG FUNCTIONS
//
internal_packet BLEduino::read_debug(){

  if (_free_node_index != 0)
    --_free_node_index;

  return _receive_buffer[_receive_buffer_head];
}

void BLEduino::print_all(){
  Serial.print("Buffer Head = "); Serial.println(_receive_buffer_head);
  Serial.print("Buffer Tail = "); Serial.println(_receive_buffer_tail);

  for(uint8_t i = 0; i < _SS_MAX_RX_BUFF; i++){
    Serial.print("Packet["); Serial.print(i); Serial.print("].pipe = ");
    Serial.println(_receive_buffer[i].pipe);

    Serial.print("Packet["); Serial.print(i); Serial.print("].size = ");
    Serial.println(_receive_buffer[i].size);

    Serial.print("Packet["); Serial.print(i); Serial.print("].next = ");
    Serial.println(_receive_buffer[i].next);

    for(uint8_t k = 0; k < 20; k++){
      Serial.print("Packet["); Serial.print(i); Serial.print("].data["); Serial.print(k); Serial.print("] = ");
      Serial.println(_receive_buffer[i].data[k], HEX);
    }

    Serial.println("");
    delay(10);

  }

  Serial.println("////////////////////////////");
}

//Print address of all packets in order, from head to tail.

void BLEduino::print_all_list(){

  if(_free_node_index == 0){
    Serial.println("Buffer empty.  Nothing to print");
  }

  else{
    //print head
    Serial.print(_receive_buffer_head); 

    for(uint8_t i = _receive_buffer[_receive_buffer_head].next; i != _receive_buffer_head;){
      Serial.print(" -> ");
      Serial.print(i);
      i = _receive_buffer[i].next;
    }

    Serial.println("");
  }

}

uint8_t BLEduino::return_head(){
  return _receive_buffer_head;
}

uint8_t BLEduino::return_tail(){
  return _receive_buffer_tail;
}

BLEPacket BLEduino::read(uint8_t pipe){

  // Empty buffer?
  if (_free_node_index == 0)
    return EMPTY_BLEPACKET; //RETURN POINTER TO STRUCT WITH EVERYTHING = TO 0

  //search all nodes in use for pipe
  uint8_t previous = _receive_buffer_tail;
  uint8_t current = _receive_buffer_head;

  uint8_t packet_count = _free_node_index;

  for(uint8_t i = 0; i < packet_count; i++){
    if(_receive_buffer[current].pipe == pipe){

      _buffer_overflow = 0;

      // Read found packet
      BLEPacket d;
      d.pipe    = _receive_buffer[current].pipe;
      d.length  = _receive_buffer[current].size;
      d.data    = _receive_buffer[current].data;

      //if head or tail, deal with that
      if(current == _receive_buffer_head){
        _receive_buffer_head = _receive_buffer[current].next;
      }

      if(current == _receive_buffer_tail){
        _receive_buffer_tail = previous;
      }

      //stitch
      _receive_buffer[previous].next = _receive_buffer[current].next;

      //add to _receive_buffer_free_nodes & //update _free_node_index
      _receive_buffer_free_nodes[--_free_node_index] = current;


      return d;
    }

    previous = current;
    current = _receive_buffer[current].next;
  }

  return EMPTY_BLEPACKET;
}

uint8_t BLEduino::available()
{
  if (!isListening())
    return 0;

  return _free_node_index;
}

uint8_t BLEduino::available(uint8_t pipe)
{
  uint8_t available = 0;

  //Search for node with pipe
  uint8_t current = _receive_buffer_head;
  for(uint8_t i = 0; i < _free_node_index; i++){
    if(_receive_buffer[current].pipe == pipe){
      available++;
    }

    current = _receive_buffer[current].next;
  }

  return available;
}

//send packet
uint8_t BLEduino::sendData(uint8_t pipe, uint8_t* data, uint8_t size){
  //Packet structure [data-length | data-byte (0xDA) | pipe | data(...) ]
  active_object->write(size + 2);
  active_object->write(0xDA);
  active_object->write(pipe);

  for(uint8_t i = 0; i < size; i++){
    active_object->write(data[i]);
  }

  return 1;
}

uint8_t BLEduino::sendData(uint8_t pipe, uint8_t data){
  //Packet structure [data-length | data-byte (0xDA) | pipe | data ]
  active_object->write(3);
  active_object->write(0xDA);
  active_object->write(pipe);
  active_object->write(data);

  return 1;
}

uint8_t BLEduino::sendCommand(uint8_t command){

  switch(command){

    case COMMAND_RESET:
      active_object->write(0x02);
      active_object->write(0xC0);
      active_object->write(0x00);
      _ble_is_connected = false;
      delay(2500);
    break;

    case COMMAND_DISCONNECT:
      active_object->write(0x02);
      active_object->write(0xC0);
      active_object->write(0xD1);
      _ble_is_connected = false;
    break;

    case COMMAND_IS_CONNECTED:
      active_object->write(0x02);
      active_object->write(0xC0);
      active_object->write(0x1C);
    break;

    case COMMAND_LED_ON:
      active_object->write(0x03);
      active_object->write(0xC0);
      active_object->write(0xB7);
      active_object->write(0x01);
    break;

    case COMMAND_LED_OFF:
      active_object->write(0x03);
      active_object->write(0xC0);
      active_object->write(0xB7);
      active_object->write(0x00);
    break;

    //4800, 9600, 14400, 19200, 28800, 31250, 38400, 57600
    case COMMAND_BAUD_RATE_9600:
      active_object->write(0x03);
      active_object->write(0xC0);
      active_object->write(0xCB);
      active_object->write(0x01);
    break;

    //9600, 14400, 19200, 28800, 31250, 38400, 57600
    case COMMAND_BAUD_RATE_14400:
      active_object->write(0x03);
      active_object->write(0xC0);
      active_object->write(0xCB);
      active_object->write(0x02);
    break;

    //9600, 14400, 19200, 28800, 31250, 38400, 57600
    case COMMAND_BAUD_RATE_19200:
      active_object->write(0x03);
      active_object->write(0xC0);
      active_object->write(0xCB);
      active_object->write(0x03);
    break;

    //9600, 14400, 19200, 28800, 31250, 38400, 57600
    case COMMAND_BAUD_RATE_28800:
      active_object->write(0x03);
      active_object->write(0xC0);
      active_object->write(0xCB);
      active_object->write(0x04);
    break;

    //9600, 14400, 19200, 28800, 31250, 38400, 57600
    case COMMAND_BAUD_RATE_31250:
      active_object->write(0x03);
      active_object->write(0xC0);
      active_object->write(0xCB);
      active_object->write(0x05);
    break;

    //9600, 14400, 19200, 28800, 31250, 38400, 57600
    case COMMAND_BAUD_RATE_38400:
      active_object->write(0x03);
      active_object->write(0xC0);
      active_object->write(0xCB);
      active_object->write(0x06);
    break;

    //9600, 14400, 19200, 28800, 31250, 38400, 57600
    case COMMAND_BAUD_RATE_57600:
      active_object->write(0x03);
      active_object->write(0xC0);
      active_object->write(0xCB);
      active_object->write(0x07);
    break;

    case COMMAND_SLEEP_TOGGLE: //DANGER DANGER HIGH VOLTAGE
      active_object->write(0x02);
      active_object->write(0xC0);
      active_object->write(0x5E);
    break;

    case COMMAND_CHANGE_DEVICE_NAME:
      //Figure this one out, since it needs a parameter
      //For now, this is dealt with in the read_packet function.
    break;

  }

  delay(100);

  return 1;
}

//write single byte at 31250 baud
uint8_t BLEduino::write(uint8_t b)
{

  uint8_t oldSREG = SREG;
  cli();  // turn off interrupts for a clean txmit

  //PORTC = PORTC | B10000000; //Debug pulse pin 3

  // Write the start bit
  tx_pin_write(LOW);
  tunedDelay(table_a.tx_delay + 5);

  if(_inverse_logic){
    //
  }

  else{

    // Write each of the 8 bits
    for (byte mask = 0x01; mask; mask <<= 1)
    {
      if (b & mask) // choose bit
        tx_pin_write(HIGH); // send 1
      else
        tx_pin_write(LOW); // send 0
    
      tunedDelay(table_a.tx_delay);
    }

  }

  tx_pin_write(HIGH); // restore pin to natural state
  

  SREG = oldSREG; // turn interrupts back on
  tunedDelay(table_a.tx_delay);

  //PORTC = PORTC & B01111111; //Debug pulse pin 3 end

  return 1;
}

//Writes at 57600 baud
uint8_t BLEduino::write_fast(uint8_t b){
  uint8_t oldSREG = SREG;
  cli();  // turn off interrupts for a clean txmit

  //PORTC = PORTC | B10000000; //Debug pulse pin 3

  // Write the start bit
  tx_pin_write(LOW);
  tunedDelay(33 + 5);

  if(_inverse_logic){
    //
  }

  else{

    // Write each of the 8 bits
    for (byte mask = 0x01; mask; mask <<= 1)
    {
      if (b & mask) // choose bit
        tx_pin_write(HIGH); // send 1
      else
        tx_pin_write(LOW); // send 0
    
      tunedDelay(33);
    }

  }

  tx_pin_write(HIGH); // restore pin to natural state
  

  SREG = oldSREG; // turn interrupts back on
  tunedDelay(33);

  //PORTC = PORTC & B01111111; //Debug pulse pin 3 end

  return 1;
}

void BLEduino::flush()
{
  if (!isListening())
    return;

  uint8_t oldSREG = SREG;
  cli();

  //Initialize free_nodes list
  for(uint8_t i = 0; i < _SS_MAX_RX_BUFF; i++){
    _receive_buffer_free_nodes[i] = i;
  }

  _receive_buffer_head = _receive_buffer_tail = _free_node_index = _buffer_overflow = 0;
  SREG = oldSREG;
}

void BLEduino::flush(uint8_t pipe){

  if (!isListening())
    return;

  uint8_t oldSREG = SREG;
  cli();

  //Search for node with pipe
  uint8_t previous = _receive_buffer_tail;
  uint8_t current = _receive_buffer_head;

  uint8_t packet_count = _free_node_index;

  for(uint8_t i = 0; i < packet_count; i++){ //scan

    if(_receive_buffer[current].pipe == pipe){ //if I find a node with "pipe"
      
      _buffer_overflow = 0;

      //if head or tail, deal with that
      if(current == _receive_buffer_head){
        _receive_buffer_head = _receive_buffer[current].next;
      }

      if(current == _receive_buffer_tail){
        _receive_buffer_tail = previous;
      }

      //stitch
      _receive_buffer[previous].next = _receive_buffer[current].next;

      //add to _receive_buffer_free_nodes & //update _free_node_index
      _receive_buffer_free_nodes[--_free_node_index] = current;
      
    }

    else{
      previous = current;
    }
    
    current = _receive_buffer[current].next;
  }

  SREG = oldSREG;
}

BLEPacket BLEduino::peek()
{
  // Empty buffer?
  if (_free_node_index == 0)
    return EMPTY_BLEPACKET;

  // Read from "head"
  BLEPacket d;
  d.pipe    = _receive_buffer[_receive_buffer_head].pipe;
  d.length  = _receive_buffer[_receive_buffer_head].size;
  d.data    = _receive_buffer[_receive_buffer_head].data;

  return d;
}

BLEPacket BLEduino::peek(uint8_t pipe){

  // Empty buffer?
  if (_free_node_index == 0)
    return EMPTY_BLEPACKET;

  //Search for node with pipe
  uint8_t current = _receive_buffer_head;
  for(uint8_t i = 0; i < _free_node_index; i++){
    if(_receive_buffer[current].pipe == pipe){
      // Read from "head"
      BLEPacket d;
      d.pipe    = _receive_buffer[current].pipe;
      d.length  = _receive_buffer[current].size;
      d.data    = _receive_buffer[current].data;

      return d;
    }

    current = _receive_buffer[current].next;
  }
  return EMPTY_BLEPACKET;
}
