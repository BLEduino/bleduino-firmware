//BLE Bridge B Module Code
//Send commands to another BLEduino (using the phone as a hub).
#include <BLEduino.h>

BLEduino BLE;    

//This is the name of this BLEduino
//Do not use 0xFF as the name.  It is reserved.
char my_name = 'B'; 

//This is the name of the BLEduino that I will send data to (and sends me data)
//Do not use 0xFF as the name.  It is reserved.
char friend_name = 'A'; 

boolean led_state; 

void setup(){
  BLE.begin(); //Initialize BLE object
  BLE.sendCommand(COMMAND_RESET); //Start advertising BLEduino

  BLE.sendData(BRIDGE_ID_SEND, my_name);
}

void loop(){

  //Using pipes is optional
  //This will return the number of packets available from the BRIDGE pipe
  //Using BLE.available() will return all data returned, regardless of pipe.
  if(BLE.available(BRIDGE_READ)){ //Read command sent from other BLEduino using BLE Bridge
   
    //Read packet
    //The parameter is also optional here.  
    //BLE.read() will return the last packet received, regardless of pipe.
    BLEPacket packet = BLE.read(BRIDGE_READ);
    
    //Parse packet
    uint8_t length = packet.length;
    uint8_t * data = packet.data;

    /***************************************************
    Packet Structure for BLE Bridge data
    ||      0      |      0      | ... |     N-1     ||
    || Destination | Character 1 | ... | Character N ||

    Destination = friend_name
    ***************************************************/

    //uint8_t who_sent_this = data[0]; //not used here

    switch(data[1]){
      case '0':
        //toggle BLEduino LED 
        led_state ^= 1;
        pinMode(BLEDUINO_LED, OUTPUT);
        digitalWrite(BLEDUINO_LED, led_state);
      break;

      case '1':
      //Add more commands here.
      break;

    }

  }
  
}