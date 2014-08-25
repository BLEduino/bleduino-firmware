//Vehicle (RC Car) Module Code
//Control an RC vehicle from your phone.
#include <BLEduino.h>

BLEduino BLE;       

#define SPEED_A 3  //PWM control for motor outputs 1 and 2 is on digital pin 3
#define SPEED_B BD11 //PWM control for motor outputs 3 and 4 is on digital pin 11
#define DIR_A BD12 //dir control for motor outputs 1 and 2 is on digital pin 12
#define DIR_B BD13 //dir control for motor outputs 3 and 4 is on digital pin 13
#define FORWARD 0
#define REVERSE 1

void setup(){
  BLE.begin(); //Initialize BLE object
  BLE.sendCommand(COMMAND_RESET); //Start advertising BLEduino
  
  Serial.begin(57600);
  
  pinMode(SPEED_A, OUTPUT);  //Set control pins to be outputs
  pinMode(SPEED_B, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  
  // Initialize all pins as low:
  digitalWrite(SPEED_A, LOW);
  digitalWrite(SPEED_B, LOW);
  digitalWrite(DIR_A, LOW);
  digitalWrite(DIR_B, LOW);
}

void loop(){
  
  //Using pipes is optional
  //This will return the number of packets available from the VEHICLE pipe
  //Using BLE.available() will return all data returned, regardless of pipe.
  if(BLE.available(VEHICLE_READ)){
    
    //Read packet
    //The parameter is also optional here.  
    //BLE.read() will return the last packet received, regardless of pipe.
    BLEPacket packet = BLE.read(VEHICLE_READ);
    
    //Parse packet
    uint8_t length = packet.length;
    uint8_t * data = packet.data;
    /*************************************************************
    Packet Structure for Vehicle data
    ||     0    |  1  |   2  |   3   ||
    || Throttle | Yaw | Roll | Pitch ||

    We only need throttle and yaw for our example.
    All values are unsigned so 0 = min | 16 = neutral | 32 = max
    ***************************************************************/

    int throttle = data[0] - 15;  //centralize values to 0
    int yaw = data[1] - 15;       //centralize values to 0
    //byte roll = data[2];        //not used
    //byte pitch = data[3];       //not used

    drive(throttle, yaw);

  }
}

void drive(int throttle, int yaw){
  int right_wheel = 0;
  int left_wheel = 0;
  int dir = 0; //Low = forward //High = reverse

  if(yaw > 0){ //steer right
    right_wheel = 127;

    if(throttle < 0){ //going forward
      left_wheel = 255;
      dir = FORWARD;
    }

    else if(throttle > 0){ //going in reverse
      left_wheel = 255;
      dir = REVERSE;
    }
    
    else if(throttle == 0){
      left_wheel = 0;
      right_wheel = 0;
    }
  }

  else if(yaw < 0){ //steer left
    left_wheel = 127;

    if(throttle < 0){ //going forward
      right_wheel = 255;
      dir = FORWARD;
    }

    else if(throttle > 0){ //going in reverse
      right_wheel = 255;
      dir = REVERSE;
    }
    
    else if(throttle == 0){
      left_wheel = 0;
      right_wheel = 0;
    }
  }
  
  else if(yaw == 0){
    if(throttle < 0){ //going forward
      right_wheel = 255;
      left_wheel = 255;
      dir = FORWARD;
    }

    else if(throttle > 0){ //going in reverse
      right_wheel = 255;
      left_wheel = 255;
      dir = REVERSE;
    }
    
    else if(throttle == 0){
      left_wheel = 0;
      right_wheel = 0;
    }
  }

  digitalWrite(DIR_A, dir);
  digitalWrite(DIR_B, dir);  
  
  analogWrite(SPEED_A, left_wheel);
  analogWrite(SPEED_B, right_wheel);
}