//LCD Module Code
//Write string on phone -> Write string on LCD
#include <BLEduino.h>
#include <LiquidCrystal.h>

#define STRING_START 0
#define STRING_PENDING 1
#define STRING_END 2

#define LCD_LINES 2
#define LCD_CHARACTERS 16

BLEduino BLE;
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

byte line = 0;
byte character = 0;

void setup(){
  BLE.begin(); //Initialize BLE object
  BLE.sendCommand(COMMAND_RESET); //Start advertising BLEduino
  
  lcd.begin(16, 2);
  lcd.print("Hello Universe");
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
    byte length = packet.length;
    byte * data = packet.data;
    /**************************************************
    Packet Structure for LCD data
    ||      0      |      1      | ... |       N       ||
    || Indentifier | Character 0 | ... | Character N-1 ||
    
    Identifier = 0, 1, 2 (beginning, pending, end)
    ***************************************************/
    
    if(!(data[0] == STRING_PENDING || data[0] == STRING_END)){
      lcd.clear();
      line = 0;
      character = 0;
    }
    
    //Write message to LCD
    for(byte i = 1; i < length; i++){
      
      lcd.print((char) data[i]);
      character++;
      
      if(character == LCD_CHARACTERS){ 
        line++; 
        character = 0;
        lcd.setCursor(character, line);
      }
      
      if(line == LCD_LINES){
        line = 0;
        lcd.setCursor(character, line);
      }
    }
  }
}
