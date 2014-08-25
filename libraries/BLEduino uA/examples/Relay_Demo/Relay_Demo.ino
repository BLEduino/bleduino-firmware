//Relay Module Code
//Write string on phone -> Write string to computer via HID simulation
#include <BLEduino.h>

BLEduino BLE;

byte relay_pin = 13;

void setup(){
  BLE.begin(); //Initialize BLE object
  BLE.sendCommand(COMMAND_RESET); //Start advertising BLEduino
  
  pinMode(relay_pin, OUTPUT);
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
    uint8_t length = packet.length; //not used here
    uint8_t * data = packet.data;
    /**************************************************
    Packet Structure for Relay data
    ||      0               |     1    |      2      ||
    || Relay Pin (Optional) | Not Used | Relay Value ||
    
    Relay pin is not used in this example.
    ***************************************************/
    
    //Change relay state
    digitalWrite(relay_pin, data[2]);

  }
}
