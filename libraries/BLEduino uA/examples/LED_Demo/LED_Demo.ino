//Relay Module Code
//Write string on phone -> Write string to computer via HID simulation
#include <BLEduino.h>

BLEduino BLE;

byte relay_pin = 3;

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
        //uint8_t length = packet.length; //not used here
        uint8_t * data = packet.data;
        /************************************
        Packet Structure for LED data
        ||    0    |     1    |      2    ||
        || LED Pin | Not Used | Pin State ||

        Using firmata pipe
        *************************************/

        //Parse LED data
        byte led_pin = data[0];
        byte led_state = data[2];

        pinMode(led_pin, OUTPUT);
        digitalWrite(led_pin, led_state);

    }
}
