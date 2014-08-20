//Notification Module Code
//Write a string into the Serial monitor and send it to your phone.
#include <BLEduino.h>

#define WRITE_DIGITAL 0
#define READ_DIGITAL 1
#define WRITE_PWM 2
#define READ_ANALOG 3

BLEduino BLE;       

void setup(){
  BLE.begin(); //Initialize BLE object
  BLE.sendCommand(COMMAND_RESET); //Start advertising BLEduino
  
  Serial.begin(57600);

  while(!Serial){;} //Waiting for arduino serial monitor to be opened
  
  Serial.println("Type notification here...");
}

void loop(){
  if(Serial.available()){

    unsigned char notification[20]; //the max size of the notification
    byte length = Serial.available();

    for(int i = 0; Serial.available(); i++){
      notification[i] = Serial.read();
    }
    //notification[length] = '\0';
    
   /**************************************************
    Packet Structure for Notification data
    ||      0      |      1      | ... |      N      ||
    || Character 0 | Character 1 | ... | Character N ||

    N -> Max = 20
    Pipe: UART
    ***************************************************/

    BLE.sendData(NOTIFICATION_SEND, notification, length);
    Serial.println("Notification Sent");

  }
}