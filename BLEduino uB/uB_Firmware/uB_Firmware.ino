/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision$
 */
 
/**
 * My project template
 */

/** @defgroup my_project my_project
@{
@ingroup projects
@brief Empty project that can be used as a template for new projects.

@details
This project is a firmware template for new projects. 
The project will run correctly in its current state, but does nothing. 
With this project you have a starting point for adding your own application functionality.

The following instructions describe the steps to be made on the Windows PC:

 -# Install the Master Control Panel on your computer. Connect the Master Emulator 
    (nRF2739) and make sure the hardware drivers are installed.
    
 */
#include <SPI.h>
#include <avr/pgmspace.h>
#include <ble_system.h>
#include <lib_aci.h>
#include <aci_setup.h>

//SLEEP
#include <avr/power.h>
#include <avr/sleep.h>

/**
Put the nRF8001 setup in the RAM of the nRF8001.
*/
#include "services.h"
/**
Include the services_lock.h to put the setup in the OTP memory of the nRF8001.
This would mean that the setup cannot be changed once put in.
However this removes the need to do the setup of the nRF8001 on every reset.
*/


#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
    static services_pipe_type_mapping_t
        services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
    #define NUMBER_OF_PIPES 0
    static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif
static hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;

//@todo have an aci_struct that will contain 
// total initial credits
// current credit
// current state of the aci (setup/standby/active/sleep)
// open remote pipe pending
// close remote pipe pending
// Current pipe available bitmap
// Current pipe closed bitmap
// Current connection interval, slave latency and link supervision timeout
// Current State of the the GATT client (Service Discovery)
// Status of the bond (R) Peer address
static struct aci_state_t aci_state;
static hal_aci_evt_t aci_data;
static hal_aci_data_t aci_cmd;

#define BLE_LED 6


/***********************************************************************************
SETUP
***********************************************************************************/
void setup(void)
{ 
  
  pinMode(BLE_LED, OUTPUT);
  //digitalWrite(BLE_LED,HIGH); //DEBUG
  
  //delay(1000); //DEBUG
  
  digitalWrite(BLE_LED, LOW);
  //debug
  //digitalWrite(BLE_LED, HIGH);
  //delay(10000);
  //digitalWrite(BLE_LED, LOW);
  ///debug
  
  //Start communication with µA
  Serial.begin(57600);
  //Serial.println(F("Arduino setup"));
  
  if (NULL != services_pipe_type_mapping)
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
  }
  else
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
  }
  aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs         = setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;
  
  //BLEduino won't start advertising until you send call lib_aci_init(&aci_state)
}

/***********************************************************************************
LOOP
***********************************************************************************/

//boolean start_device = false;
boolean device_started = false;
boolean device_connected = false;
boolean sleep_enable = 0;
uint8_t sleep;

void loop(){
  
   sleep = 0;
  
  // We enter the if statement only when there is a ACI event available to be processed
  if (device_started && lib_aci_event_get(&aci_state, &aci_data)){
    sleep++;
    aci_loop();
  }
  
  
  if(Serial.available()){
    read_from_A();
    sleep++;
  }
  
  if(sleep_enable == 1 && sleep == 0){ 
    //SLEEP HERE
    sleep_now(); //DANGER DANGER
  }
  
}

/***********************************************************************************
ACI_LOOP
***********************************************************************************/
void aci_loop(){
  
    aci_evt_t * aci_evt;
    
    aci_evt = &aci_data.evt;    
    switch(aci_evt->evt_opcode)
    {
        /**
        As soon as you reset the nRF8001 you will get an ACI Device Started Event
        */
        case ACI_EVT_DEVICE_STARTED:
        {          
          //aci_state.data_credit_available = aci_evt->params.device_started.credit_available;
          switch(aci_evt->params.device_started.device_mode)
          {
            case ACI_DEVICE_SETUP:
            /**
            When the device is in the setup mode
            */
            //Serial.println(F("Evt Device Started: Setup"));
            if (ACI_STATUS_TRANSACTION_COMPLETE != do_aci_setup(&aci_state))
            {
              //Serial.println(F("Error in ACI Setup"));
            }
            //Serial.println(F("Setup Done!"));
            break;
            
            case ACI_DEVICE_STANDBY:
              //Serial.println(F("Evt Device Started: Standby"));
              lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
              //Serial.println(F("Advertising started"));
              break;
          }
        }
        break; //ACI Device Started Event
        
      case ACI_EVT_CMD_RSP:
        //If this event ever happens, notify µA so it can do something with the LED's
        //If an ACI command response event comes with an error -> stop
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
        {
          //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
          //Serial.print(F("ACI Command "));
          //Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
          //Serial.println(F("Evt Cmd respone: Error. Arduino is in an while(1); loop"));
          
          int brightness = 0;    // how bright the LED is
          int fadeAmount = 5;    // how many points to fade the LED by
          
          while (1){
            // set the brightness of pin 9:
            analogWrite(BLE_LED, brightness);    
          
            // change the brightness for next time through the loop:
            brightness = brightness + fadeAmount;
          
            // reverse the direction of the fading at the ends of the fade: 
            if (brightness == 0 || brightness == 255) {
              fadeAmount = -fadeAmount ; 
            }     
            // wait for 30 milliseconds to see the dimming effect    
            delay(30);                             
          }
        }
        break;
        
      case ACI_EVT_CONNECTED:
        digitalWrite(BLE_LED, HIGH);
        device_connected = true;
        send_connect_status();
        
        //send connected status to µA TODO
        
        //Serial.println(F("Evt Connected"));
        //aci_state.data_credit_available = aci_state.data_credit_total; 
        break;
        
      case ACI_EVT_PIPE_STATUS:
        //Serial.println(F("Evt Pipe Status"));
        break;
        
      case ACI_EVT_DISCONNECTED:
        digitalWrite(BLE_LED, LOW);
        device_connected = false;
        send_connect_status();
        //send disconnected status to µA TODO
        
        //Serial.println(F("Evt Disconnected/Advertising timed out"));
        lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
        //Serial.println(F("Advertising started"));        
        break;
        
      case ACI_EVT_DATA_CREDIT:
      //aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
      break;
        
      case ACI_EVT_PIPE_ERROR:
        //See the appendix in the nRF8001 Product Specication for details on the error codes
        //Serial.print(F("ACI Evt Pipe Error: Pipe #:"));
        //Serial.print(aci_evt->params.pipe_error.pipe_number, DEC);
        //Serial.print(F("  Pipe Error Code: 0x"));
        //Serial.println(aci_evt->params.pipe_error.error_code, HEX);
                
        //Increment the credit available as the data packet was not sent
        //aci_state.data_credit_available++;
        break;  
        
      case ACI_EVT_DATA_RECEIVED:
        //Serial.print(F("Pipe #: 0x"));
        //Serial.println(aci_evt->params.data_received.rx_data.pipe_number, HEX);
        
        parse_BLE_data(aci_evt);
        break; 
     
     default: //I don't care about anything here, I'm sleepy.
       sleep--;
    }
}

/***********************************************************************************
PARSE_BLE_DATA from BLE
***********************************************************************************/
void parse_BLE_data(aci_evt_t * aci_evt){

  //Data was just received, check what pipe it came from.
  switch(aci_evt->params.data_received.rx_data.pipe_number){
    
    case PIPE_CONTROLLER__BUTTON_ACTION_RX:
    case PIPE_CONTROLLER__BUTTON_ACTION_RX_ACK_AUTO:
      //Debug
      //Serial.println("Data Received on pipe: PIPE_CONTROLLER__BUTTON_ACTION_RX");
      //Parse virtual controller data
      uint8_t controller_packet_in[5];
      controller_packet_in[0] = aci_evt->params.data_received.rx_data.pipe_number; //pipe number
      controller_packet_in[1] = 3; //4 bytes not counting length byte.
      controller_packet_in[2] = aci_evt->params.data_received.rx_data.aci_data[0]; //button name
      controller_packet_in[3] = aci_evt->params.data_received.rx_data.aci_data[1]; //button state [0 pressed/1 unpressed]
      controller_packet_in[4] = aci_evt->params.data_received.rx_data.aci_data[2]; //joystick value [0 min-128 neutral-255 max]
      
      //Send to µA
      Serial.write(controller_packet_in, 5);
      break;///////////////////////////////////////////
    
    case PIPE_FIRMATA_FIRMATA_COMMAND_RX:
    case PIPE_FIRMATA_FIRMATA_COMMAND_RX_ACK_AUTO:
      //Debug
      //Serial.println("Data Received on pipe: PIPE_FIRMATA_FIRMATA_COMMAND_RX");
      //Parse firmata data
      uint8_t firmata_packet_in[5];
      firmata_packet_in[0] = aci_evt->params.data_received.rx_data.pipe_number; //pipe number
      firmata_packet_in[1] = 3; //data length -> 4 bytes not counting length byte
      firmata_packet_in[2] = aci_evt->params.data_received.rx_data.aci_data[0]; //pin number
      firmata_packet_in[3] = aci_evt->params.data_received.rx_data.aci_data[1]; //pin state[0-output, 1-input, 2-pwm, 3-analog]
      firmata_packet_in[4] = aci_evt->params.data_received.rx_data.aci_data[2]; //value[0-255]
      
      //Send to µA
      Serial.write(firmata_packet_in, 5);
      break;//////////////////////////////////////////
    
    case PIPE_UART_RX_RX:
    case PIPE_UART_RX_RX_ACK_AUTO:
      
      //Debug
      //Serial.println("Data Received on pipe: PIPE_UART_RX_RX");
      
    {//new scope (switch-case fix)
      
      uint8_t ble_packet_length = (aci_evt->len) - 2;
      uint8_t uart_packet_in[ble_packet_length + 2];  
      
      uart_packet_in[0] = aci_evt->params.data_received.rx_data.pipe_number; //pipe number
      uart_packet_in[1] = ble_packet_length; //data length
      
      int i = 0;
      for(i = 0; i < ble_packet_length; i++){
          uart_packet_in[i+2] = aci_evt->params.data_received.rx_data.aci_data[i]; //data...
      }
        
      Serial.write(uart_packet_in, ble_packet_length + 2);
    }
      break;//////////////////////////////////////////
    
    case PIPE_NOTIFICATION_NOTIFICATION_ATTRIBUTES_RX:
    case PIPE_NOTIFICATION_NOTIFICATION_ATTRIBUTES_RX_ACK_AUTO:
      //NOT USED
      //Parse notification.
      //Send to µA.
    break;//////////////////////////////////////////
    
    case PIPE_BLE_BRIDGE_DEVICE_ID_RX:
    case PIPE_BLE_BRIDGE_DEVICE_ID_RX_ACK_AUTO:
      //NOT USED
      //Read new device name for bridge interaction.
      //Set new BLEduino name to µA.
    break;//////////////////////////////////////////
    
    case PIPE_BLE_BRIDGE_BRIDGE_RX_RX:
    case PIPE_BLE_BRIDGE_BRIDGE_RX_RX_ACK_AUTO:
      //Debug
      //Serial.println("Data Received on pipe: PIPE_BLE_BRIDGE_BRIDGE_RX_RX");
      
      //[source, destination, first 0-transit 1-last 2, data...]
      {//new scope (switch-case fix)
      uint8_t ble_packet_length = (aci_evt->len) - 2;
      uint8_t bridge_packet_in[ble_packet_length + 2];  
      
      bridge_packet_in[0] = aci_evt->params.data_received.rx_data.pipe_number; //pipe number
      bridge_packet_in[1] = ble_packet_length; //data length
      //bridge_packet_in[1] = aci_evt->params.data_received.rx_data.aci_data[0]; //source (dude who sent it)
      //bridge_packet_in[2] = aci_evt->params.data_received.rx_data.aci_data[1]; //destination (me)
      //bridge_packet_in[3] = aci_evt->params.data_received.rx_data.aci_data[2]; //first 0, transit 1, last 2
      
      int i = 0;
      for(i = 0; i < ble_packet_length; i++){
          bridge_packet_in[i+2] = aci_evt->params.data_received.rx_data.aci_data[i]; //data...
      }
        
      Serial.write(bridge_packet_in, ble_packet_length + 2);
    }
      //Read data from another BLEduino.  Sent raw, just like the UART pipe.
      //Send raw data to µA.  First byte will probably be the ID of the BLEduino that sent the data.
    break;//////////////////////////////////////////
    
    case PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_RX:
    case PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_RX_ACK_AUTO:
      //Debug
      //Serial.println("Data Received on pipe: PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_RX");
      
      //Parse vehicle control data.
      //throttle, yaw, roll, pitch
      uint8_t vehicle_motion_packet_in[6];
      vehicle_motion_packet_in[0] = aci_evt->params.data_received.rx_data.pipe_number; //pipe number
      vehicle_motion_packet_in[1] = 4; //data length
      vehicle_motion_packet_in[2] = aci_evt->params.data_received.rx_data.aci_data[0]; //throttle
      vehicle_motion_packet_in[3] = aci_evt->params.data_received.rx_data.aci_data[1]; //yaw
      vehicle_motion_packet_in[4] = aci_evt->params.data_received.rx_data.aci_data[2]; //roll
      vehicle_motion_packet_in[5] = aci_evt->params.data_received.rx_data.aci_data[3]; //pitch
      
      //Send to µA
      Serial.write(vehicle_motion_packet_in, 6);
    break;//////////////////////////////////////////
  }
}

/***********************************************************************************
READ_FROM_A
***********************************************************************************/
uint8_t inBuf[28];

void read_from_A(){
  
  int index = 0;
  int data_length = Serial.read();
  
  //read entire packet first
  Serial.readBytes((char *)&inBuf[0], data_length);
  
  byte first_byte = inBuf[0];
  
  //if first byte is 0xC0 then it's a command
  if(first_byte == 0xC0){
    //commands:
    switch (inBuf[1]) { 
      
      case 0x00: //reset the nRF8001
      {
        digitalWrite(BLE_LED, LOW);
        lib_aci_init(&aci_state);
        device_started = true;
        //Serial.println("RESET");
      }
        break;
        
      case 0xD1:
     { //Disconnect
        lib_aci_radio_reset();
        digitalWrite(BLE_LED, LOW);
        device_started = false;
        device_connected = false;
     }
        break;
        
      case 0x1C: //is connected
      {
        send_connect_status();
      }
        break;
      
      case 0xB7: //hardcode BLE led on-off [0xC0, 0xB7, 0x01 || 0x00]
      {
        //hardcode that shit
        digitalWrite(BLE_LED, inBuf[2]);
        //Serial.println("LED TOGGLED!");
      }
        break;
      
      case 0xCB: //change baud-rate 
      {
        //[0xC0, 0xCB, baud-options: 0-4800, 1-9600, 2-14400, 3-19200, 4-28800, 5-31250, 6-38400, 7-57600]
        int baud_options[] = {4800, 9600, 14400, 19200, 28800, 31250, 38400, 57600}; //CHANGE TO UNSIGNED LONG
        
        Serial.begin(baud_options[inBuf[2]]);
      }
        break;
      
      case 0x71: //change timing? [0xC0, 0x71, ...]
        //change timing and shit
        //lib_aci_change_timing_GAP_PPCP();
        break;
      
      case 0x7E: //test mode [0xC0, 0x7E]
        //toggle nRF8001 in & out of test mode
        //research this better
        break;
        
      case 0x5E: //sleep enable
        sleep_enable ^= 1; 
      break;
        
      
      case 0xCD: //change device name "PIPE_GAP_DEVICE_NAME_SET" [0xC0, 0xCD, ...]
      {
        //read new BLEduino name.
        /*
        uint8_t new_device_name_size = data_length - 1;
        uint8_t new_device_name[new_device_name_size];
        
        int i=0;
        for(i=0; i < new_device_name_size; i++){
          new_device_name[i] = Serial.read();
        }
        */
        
        lib_aci_set_local_data(&aci_state, PIPE_GAP_DEVICE_NAME_SET, (uint8_t *)&inBuf[2], data_length - 2);
        }
        break;
      
      //think of other commands
    }
  }
  
  //if first byte is 0xDA then it's data.
  else if(first_byte == 0xDA){
    
    uint8_t destination_pipe = inBuf[1];
    
    switch (destination_pipe) { //destination_pipe
      
      case PIPE_CONTROLLER__BUTTON_ACTION_TX:
      case PIPE_CONTROLLER__BUTTON_ACTION_TX_ACK:
        //NOT USED
        //Serial.println("Sending data to pipe: PIPE_CONTROLLER__BUTTON_ACTION_TX");
        break;
      /*
      case PIPE_FIRMATA_FIRMATA_COMMAND_TX:
      case PIPE_FIRMATA_FIRMATA_COMMAND_TX_ACK:
        //read firmata data and send on those pipes.
        //Pin_number, pin_state[1-input, 3-analog], 2 bytes[0-1023]
        
        //send that shit
        lib_aci_send_data(destination_pipe,(uint8_t *)&inBuf[2], data_length-2);
        break;
      */
      
      case PIPE_FIRMATA_FIRMATA_COMMAND_TX:
      case PIPE_FIRMATA_FIRMATA_COMMAND_TX_ACK:
        //read firmata data and send on those pipes.
        //Pin_number, pin_state[1-input, 3-analog], 2 bytes[0-1023]
      case PIPE_UART_TX_TX:
      case PIPE_UART_TX_TX_ACK:
      
      case PIPE_NOTIFICATION_NOTIFICATION_ATTRIBUTES_TX:
      case PIPE_NOTIFICATION_NOTIFICATION_ATTRIBUTES_TX_ACK:
      
      case PIPE_BLE_BRIDGE_BRIDGE_TX_TX:
      case PIPE_BLE_BRIDGE_BRIDGE_TX_TX_ACK:
      {
        /*
        //read raw UART data
        uint8_t uart_packet_out_size = Serial.available();
        uint8_t uart_packet_out[uart_packet_out_size];
        
        int i=0;
        for(i=0; i < uart_packet_out_size; i++){
          uart_packet_out[i] = Serial.read();
        }
        
        //send
        lib_aci_send_data(destination_pipe,(uint8_t *)&uart_packet_out[0] ,uart_packet_out_size);
        */
        //Serial.println("Entered Big-Ass case.  Sending data.");
        lib_aci_send_data(destination_pipe,(uint8_t *)&inBuf[2], data_length-2); 
        //Serial.println("Data Sent"); 
    }
        break;
        
     case PIPE_BLE_BRIDGE_DEVICE_ID_SET:
     {
        //read name of this bleduino (1 byte)
        
        //send
        //Serial.println("BLE_BRIDGE_DEVICE_ID_SET.  Setting.");
        lib_aci_set_local_data(&aci_state, PIPE_BLE_BRIDGE_DEVICE_ID_SET, (uint8_t *)&inBuf[2], data_length-2);
        //Serial.println("Data set");
     }
        break;
        
      /*
      case PIPE_BLE_BRIDGE_BRIDGE_TX_TX:
      case PIPE_BLE_BRIDGE_BRIDGE_TX_TX_ACK:
        //read bridge data
        //first byte is name of destination
        //send
      
      */
      
      case PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_TX:
      case PIPE_VEHICLE_MOTION_THROTTLE_YAW_ROLL_PITCH_TX_ACK:
        //NOT USED
        break;
    }
  }
  
  else{
  //if first byte is none of those -> error
  }
}

void send_connect_status(){
  uint8_t is_connected_response[3];
  is_connected_response[0] = 0; // "pipe"
  is_connected_response[1] = 1; //size of data
  is_connected_response[2] = device_connected; //data
  Serial.write(is_connected_response, 3);
}

/***********************************************************************************
SLEEP
***********************************************************************************/
void sleep_now() //HIGH VOLTAGE
{
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and 
     * wake up sources are available in which sleep modus.
     *
     * In the avr/sleep.h file, the call names of these sleep modus are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings 
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     *  the power reduction management <avr/power.h>  is described in 
     *  http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
     */  
     
  detachInterrupt(1);
  attachInterrupt(1, m_rdy_line_handle, LOW);
  
  set_sleep_mode(SLEEP_MODE_IDLE);   // sleep mode is set here

  sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin 
  
  power_adc_disable();
  //power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();
  
  sleep_mode();            // here the device is actually put to sleep!!
 
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  //sleep_disable();         // first thing after waking from sleep:
                            // disable sleep...

  power_all_enable();
}
