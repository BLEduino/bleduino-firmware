//Keyboard Module Code
//Write string on phone -> Write string to computer via HID simulation
#include <BLEduino.h>

BLEduino BLE;

void setup(){
  BLE.begin(); //Initialize BLE object
  BLE.sendCommand(COMMAND_RESET); //Start advertising BLEduino

  Keyboard.begin();
}

void loop(){

  //Using pipes is optional
  //This will return the number of packets available from the UART pipe
  //Using BLE.available() will return all data returned, regardless of pipe.
  if(BLE.available(UART_READ)){
    
    //Read packet
    //The parameter is also optional here.  
    //BLE.read() will return the last packet received, regardless of pipe.
    BLEPacket packet = BLE.read(UART_READ);
    
    //Parse packet
    uint8_t length = packet.length;
    uint8_t * data = packet.data;
    /**************************************************
    Packet Structure for Keyboard data
    ||      0      |      1      | ... |      N      ||
    || Character 0 | Character 1 | ... | Character N ||
    ***************************************************/

    for(byte i = 0; i < length; i++){
      Keyboard.write(data[i]);
    }

  }
}

