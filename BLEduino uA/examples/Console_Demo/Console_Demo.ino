//Console Module Code
//Write and receive character strings
#include <BLEduino.h>

BLEduino BLE;

void setup(){
  BLE.begin(); //Initialize BLE object
  BLE.sendCommand(COMMAND_RESET); //Start advertising BLEduino

  Serial.begin(57600);

  while(!Serial){;} //Wait for serial monitor to be opened 

  Serial.println("Console Test.  Write a message (up to 20 characters long) and press enter...");
}

void loop(){

  
  //Read message from BLEduino
  //Print in Serial Monitor

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
    
    //Print each character
    for(int i = 0; i < length; i++){
      Serial.print((char)data[i]);
    }

    Serial.println(""); //Add new line.

  }
  
  //Send message to BLEduino
  if(Serial.available()){
    
    if(!check_size()){
      return;
    }

    byte length = Serial.available();
    byte message[20];

    for(int i = 0; i < length; i++){
      message[i] = Serial.read();
    }

    BLE.sendData(UART_SEND, message, length);

    Serial.println("Message Sent");
  }
}

bool check_size(){
  if(Serial.available() > 20){
    Serial.flush();

    Serial.println("Message can be 20 characters max.");
    
    return 0;
  }
  
  return 1;
}

