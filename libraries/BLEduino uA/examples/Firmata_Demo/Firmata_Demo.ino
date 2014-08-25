//Firmata Module Code
//Control all BLEduino functions with the phone app
#include <BLEduino.h>

#define WRITE_DIGITAL 0
#define READ_DIGITAL 1
#define READ_ANALOG 2
#define WRITE_PWM 3

BLEduino BLE;       

void setup(){
  BLE.begin(); //Initialize BLE object
  BLE.sendCommand(COMMAND_RESET); //Start advertising BLEduino
}

void loop(){

  //Using pipes is optional
  //This will return the number of packets available from the FIRMATA pipe
  //Using BLE.available() will return all data returned, regardless of pipe.
  if(BLE.available(FIRMATA_READ)){
    
    //Read packet
    //The parameter is also optional here.  
    //BLE.read() will return the last packet received, regardless of pipe.
    BLEPacket packet = BLE.read(FIRMATA_READ);
    
    //Parse packet
    uint8_t length = packet.length;
    uint8_t * data = packet.data;
    /**************************************************
    Packet Structure for Vehicle data
    ||     0    |      1    |      2    ||
    || Pin Num. | Pin State | Pin Value ||

    Pin States:
    0: Output
    1: Input
    2: Analog (input)
    3: PWM (output)

    4: Beginning of sequence
    5: End of sequence
    6: Delay in seconds
    7: Delay in minutes
    ***************************************************/

    uint8_t pin_num = data[0];
    uint8_t pin_state = data[1];
    uint8_t pin_value = data[2];

    switch(pin_state){
      case WRITE_DIGITAL:
        write_digital(pin_num, pin_value);
      break;

      case READ_DIGITAL:
        read_digital(pin_num);
      break;

      case WRITE_PWM:
        write_pwm(pin_num, pin_value);
      break;

      case READ_ANALOG:
        read_analog(pin_num);
      break;
    }

  }
}

void write_digital(uint8_t _pin_num, uint8_t _pin_value){
  pinMode(_pin_num, OUTPUT);
  digitalWrite(_pin_num, _pin_value);
}

void read_digital(uint8_t _pin_num){
  pinMode(_pin_num, OUTPUT);
  digitalWrite(_pin_num, LOW);
  pinMode(_pin_num, INPUT);

  byte value[4];
  value[0] = _pin_num;
  value[1] = READ_DIGITAL;
  value[2] = 0;
  value[3] = digitalRead(_pin_num);

  BLE.sendData(FIRMATA_SEND, value, 4);
}

void write_pwm(uint8_t _pin_num, uint8_t _pin_value){
  pinMode(_pin_num, OUTPUT);
  analogWrite(_pin_num, _pin_value);
}

void read_analog(uint8_t _pin_num){
  pinMode(_pin_num, OUTPUT);
  digitalWrite(_pin_num, LOW);
  pinMode(_pin_num, INPUT);

  int value = analogRead(_pin_num);

  if(value <= 255){ 
    uint8_t packet_to_send[3];
    packet_to_send[0] = _pin_num;
    packet_to_send[1] = READ_DIGITAL;
    packet_to_send[2] = value;

    BLE.sendData(FIRMATA_SEND, packet_to_send, 3);
  }

  else{
    uint8_t byte_one = (value & 0x0300) >> 8;
    uint8_t byte_two = value & 0xFF;

    uint8_t packet_to_send[4];
    packet_to_send[0] = _pin_num;
    packet_to_send[1] = READ_DIGITAL;
    packet_to_send[2] = byte_one;
    packet_to_send[3] = byte_two;

    BLE.sendData(FIRMATA_SEND, packet_to_send, 4);
  }
}

