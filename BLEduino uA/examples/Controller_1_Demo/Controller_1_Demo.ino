//Controller-1 Module Code
//Write string on phone -> Write string to computer via HID simulation
#include <BLEduino.h>

BLEduino BLE;

//Keyboard keys to simulate. (Press "up" on app -> simulate KEY_UP_ARROW on computer)
char up = KEY_UP_ARROW;
char left = KEY_LEFT_ARROW;
char down = KEY_DOWN_ARROW;
char right = KEY_RIGHT_ARROW;

char x = 'w';
char y = 'a';
char b = 's';
char a = 'd';

char start = KEY_RETURN;
char select = KEY_RIGHT_SHIFT;

void setup(){
  BLE.begin(); //Initialize BLE object
  BLE.sendCommand(COMMAND_RESET); //Start advertising BLEduino

  Keyboard.begin();
}

void loop(){

  //Using pipes is optional
  //This will return the number of packets available from the CONTROLLER pipe
  //Using BLE.available() will return all data returned, regardless of pipe.
  if(BLE.available(CONTROLLER_READ)){
    
    //Read packet
    //The parameter is also optional here.  
    //BLE.read() will return the last packet received, regardless of pipe.
    BLEPacket packet = BLE.read(CONTROLLER_READ);
    
    //Parse packet
    //uint8_t length = packet.length; //Not used here
    uint8_t * data = packet.data;
    /***************************************
    Packet Structure for Controller-1 data
    |   0    |   1   |
    | Button | State |
    ****************************************/

    byte button = data[0];
    byte button_state = data[1];
    byte joystick_state = data[2];

    switch(button){
      case 0: //Vertical Joystick
        if(joystick_state > 0x0D){
          handle_keyboard(0, up);
          handle_keyboard(1, down); 
        }
       
        else if(joystick_state < 0x0D) { 
          handle_keyboard(0, down);
          handle_keyboard(1, up); 
        }
        
        else { 
          handle_keyboard(0, down);
          handle_keyboard(0, up);
        }
      break;

      case 1: //Horizontal Joystick
        if(joystick_state > 0x0D){
          handle_keyboard(0, left);
          handle_keyboard(1, right); 
        }
       
        else if(joystick_state < 0x0D) { 
          handle_keyboard(0, right);
          handle_keyboard(1, left); 
        }
        
        else { 
          handle_keyboard(0, left);
          handle_keyboard(0, right);
        }
      break;

      //

      case 2: handle_keyboard(button_state, y); break;

      case 3: handle_keyboard(button_state, x); break;

      case 4: handle_keyboard(button_state, a); break;

      case 5: handle_keyboard(button_state, b); break;

      ///

      case 6: handle_keyboard(button_state, start); break;

      case 7: handle_keyboard(button_state, select); break;
    }

  }
}

void handle_keyboard(char _state, char key){
  if(_state == 1){
    Keyboard.press(key);
  }

  else{
    Keyboard.release(key);
  }
}