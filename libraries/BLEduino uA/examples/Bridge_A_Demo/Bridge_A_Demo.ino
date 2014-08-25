//BLE Bridge A Module Code
//Send commands to another BLEduino (using the phone as a hub).
#include <BLEduino.h>

BLEduino BLE;       

//This is the name of this BLEduino
//Do not use 0xFF as the name.  It is reserved.
char my_name = 'A'; 

//This is the name of the BLEduino that I will send data to.
//Do not use 0xFF as the name.  It is reserved.
char friend_name = 'B'; 

void setup(){
  BLE.begin(); //Initialize BLE object
  BLE.sendCommand(COMMAND_RESET); //Start advertising BLEduino

  BLE.sendData(BRIDGE_ID_SEND, my_name);

  Serial.begin(57600);

  while(!Serial){;} 
  Serial.println("BLEduino BLE Bridge Demo");
  Serial.println("0 : Toggle BLEduino-BLED On-Off");
  Serial.println("Add more commands here...\n");
}

void loop(){
  if(Serial.available()){ //read command from Serial port

    check_size();
    
   /***************************************************
    Packet Structure for BLE Bridge data
    ||      0      |      0      | ... |     N-1     ||
    || Destination | Character 1 | ... | Character N ||

    N -> Max = 20
    Destination = friend_name
    ***************************************************/

    byte length = 2;
    byte command[length]; //the max size of the command
    
    command[0] = friend_name;
    command[1] = Serial.read();

    BLE.sendData(BRIDGE_SEND, command, 2);

   Serial.print("Sent command: "); Serial.println((char)command[1]);

  }
}

void check_size(){
  if(Serial.available() > 1){
    Serial.flush();

    Serial.println("One command at a time.");
  }
}