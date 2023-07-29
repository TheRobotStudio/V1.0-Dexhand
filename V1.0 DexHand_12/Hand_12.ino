/*   Copyright 2021 Feetech RC Model CO.,LTD
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

 
#include "SBUS.h"
#include "Servo.h"
#include "SCServo.h"

// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial1);
/*
Servo emaxservo_01;  // create servo object to control a servo
Servo emaxservo_02;
Servo emaxservo_03;
Servo emaxservo_04;
Servo emaxservo_05;
Servo emaxservo_06;
Servo emaxservo_07;
Servo emaxservo_08;
Servo emaxservo_09;
Servo emaxservo_10;
Servo emaxservo_11;
Servo emaxservo_12;
Servo emaxservo_13;
Servo emaxservo_14;
Servo emaxservo_15;
Servo emaxservo_16;
*/
Servo emaxservo_RTI;  // 26 RT I Ring Tendon Inverted, 27, 28, 29, 30, 31, 24, 48, 49, 50, 51, 52, 53, 45, 46, 47
Servo emaxservo_LTI;  // 27 LT I Little Tendon Inverted
Servo emaxservo_LON;  // 28 LO N Little Outward at Knuckle Non-Inverted
Servo emaxservo_LII;  // 29 LI I Little Inward at Knuckle Inverted
Servo emaxservo_ROI;  // 30 RO I Ring Outward
Servo emaxservo_RII;  // 31 RI Ring Inward
Servo emaxservo_TRN;  // 24 TR N Thumb Rotation Non-Inverted
Servo emaxservo_FTN;  // 48 FT N Fore Tendon Non
Servo emaxservo_MTN;  // 49 MT N Middle Tendon Non
Servo emaxservo_FII;  // 51 FI I Fore Inward Inverted
Servo emaxservo_FON;  // 50 FO N
Servo emaxservo_MON;  // 52 MO N
Servo emaxservo_MIN;  // 53 MI N
Servo emaxservo_TII;  // 45 TI I Thumb Inwards
Servo emaxservo_TTN;  // 46 TT N Thumb Tendon Non
Servo emaxservo_TON;  // 47 TO N Thumb Outwards Non



// twelve servo objects can be created on most boards.
//On the Mega, up to 12 servos can be used without interfering with PWM functionality; use of 12 to 23 motors will disable PWM on pins 11 and 12.

int pos[] ={0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0};    // variable to store positions

//int servo_pos[] ={0, 0, 0, 0,       // variable to store the servo positions for four fingers (3 powered dof each) and thumb (4 powered dof)
//                  0, 0, 0, 0,       // setting to zero causes little servo jump at start up
//                  0, 0, 0, 0,
//                  0, 0, 0, 0};
                   
int servo_pos[] ={90, 90, 90, 90,       // variable to store the servo positions for four fingers (3 powered dof each) and thumb (4 powered dof)
                  90, 90, 90, 90,
                  90, 90, 90, 90,
                  90, 90, 90, 90}; 

int wrist_pos[] = {512, 512, 512};     // default to centre

// channel, fail safe, and lost frames data
uint16_t channels[16];
bool failSafe;
bool lostFrame;

long default_counter = 1;
int default_stage = 0;


SCServo SERVO;      //Declare a case of SCServo to control the Feetechs

//
int analogPin0 = A0; // potentiometer wiper (middle terminal) connected to analog pin 0
                    // outside leads to ground and +5V
int analogPin1 = A1;  //Knuckle Yaw
int analogPin2 = A2;  //Sequence
                    
int val0 = 0;  // variable to store the value read
int val1 = 0;
int val2 = 0;
int a = 0;      //Print out angles
int b = 0;      //Servo limits for loop
int l = 0;      //for loop counter
int s = 511;      //sequence counter
bool Sense = 1;

int bytecount = 0;
int bus_start = 0;
int bus_end = 0;
int checksum = 0;
int angles[23] = {0};
byte lo;            //low byte of checksum int

int digi = 0;
int digitemp = 0;
int newmsg = 0;

int action = 0;
int prev_action = 0;

int ring_offset = 0;

//White Hand
//ID1: 380 570
//ID2: 0 1023
//ID3: 360 570
//ID4: 0 1023
//ID5: 450 640
//ID6: 0 1023 Neg
//ID7: 40 600
//ID8: 0 1023 Neg
//ID9: 400 800 Neg
//ID10: 0 1023
int Servo_Min[] = {0, 380, 0, 360, 0, 450, 0, 400, 0, 400, 0};
int Servo_Max[]= {0, 570, 1023, 570, 1023, 640, 1023, 600, 1023, 800, 1023};
int Mid_Point[10];

int wrist[2] = {512, 512};
int Servo_Order[]= {26, 27, 28, 29, 
                    30, 31, 24, 48,
                    49, 50, 51, 52,
                    53, 45, 46, 47};

void setup() {
  //The Mega and Due boards have 4 UART each
  //For the Mega  RX0 = D0, TX0 = D1
  //              RX1 = D19, TX1 = D18
  //              RX2 = D17, TX2 = D16
  //              RX3 = D15, TX3 = D14

  
  Serial.begin(115200);       //Comms with PC
  Serial3.begin(115200);      //Comms with xbee

  x8r.begin();                          // begin the SBUS communication on Serial1
  // x8r.begin(22, 23, false, 100000); // optional parameters for ESP32: RX pin, TX pin, inverse mode, baudrate
  
  Serial2.begin(1000000);     //Comms with Feetech servos RX2 = D17, TX2 = D16                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
  SERVO.pSerial = &Serial2;
/*
  emaxservo_01.attach(26);  // 26 RT I Ring Tendon Inverted, 27, 28, 29, 30, 31, 24, 48, 49, 50, 51, 52, 53, 45, 46, 47
  emaxservo_02.attach(27);  // 27 LT I Little Tendon Inverted
  emaxservo_03.attach(28);  // 28 LO N Little Outward at Knuckle Non-Inverted
  emaxservo_04.attach(29);  // 29 LI I Little Inward at Knuckle Inverted
  emaxservo_05.attach(30);  // 30 RO I Ring Outward
  emaxservo_06.attach(31);  // 31 RI Ring Inward
  emaxservo_07.attach(24);  // 24 TR N Thumb Rotation Non-Inverted
  emaxservo_08.attach(48);  // 48 FT N Fore Tendon Non
  emaxservo_09.attach(49);  // 49 MT N Middle Tendon Non
  emaxservo_10.attach(50);  // 50 FI I Fore Inward Inverted
  emaxservo_11.attach(51);  // 51 FO N Fore Outward Normal
  emaxservo_12.attach(52);  // 52 MO N Middle Outward Normal
  emaxservo_13.attach(53);  // 53 MI N Middle Inward Normal
  emaxservo_14.attach(45);  // 45 TI I Thumb Inwards Inverted
  emaxservo_15.attach(46);  // 46 TT N Thumb Tendon Non
  emaxservo_16.attach(47);  // 47 TO N Thumb Outwards Non
*/

  emaxservo_FON.attach(50);  // 50 FO N Fore Outwards Normal
  emaxservo_FII.attach(51);  // 51 FI I Fore Inward Inverted
  emaxservo_FTN.attach(48);  // 48 FT N Fore Tendon Non
   
  emaxservo_MON.attach(52);  // 52 MO N Middle Outwards Normal
  emaxservo_MIN.attach(53);  // 53 MI N Middle Inwards Normal
  emaxservo_MTN.attach(49);  // 49 MT N Middle Tendon Non
  
  emaxservo_ROI.attach(30);  // 30 RO I Ring Outward Inverted
  emaxservo_RII.attach(31);  // 31 RI I Ring Inward Inverted 
  emaxservo_RTI.attach(26);  // 26 RT I Ring Tendon Inverted
  
  emaxservo_LON.attach(28);  // 28 LO N Little Outward at Knuckle Non-Inverted
  emaxservo_LII.attach(29);  // 29 LI I Little Inward at Knuckle Inverted
  emaxservo_LTI.attach(27);  // 27 LT I Little Tendon Inverted
  
  emaxservo_TON.attach(47);  // 47 TO N Thumb Outwards Non
  emaxservo_TII.attach(45);  // 45 TI I Thumb Inwards Inverted
  emaxservo_TTN.attach(46);  // 46 TT N Thumb Tendon Non
  emaxservo_TRN.attach(24);  // 24 TR N Thumb Rotation Non-Inverted
 
  
  delay(50);                 //Try shorter than 500
}

void loop()
{
      //Serial.print("Loop ");
      

    // look for a good SBUS packet from the receiver, has dominance over Xbee signal
  if(x8r.read(&channels[0], &failSafe, &lostFrame))
  {

    // write the SBUS packet to an SBUS compatible servo
    //x8r.write(&channels[0]);
    
   

    for(int i=0; i<16; i++){ 
     //Serial.print(channels[0], DEC);
      Serial.print(i+1, DEC);
      Serial.print(" ");
      Serial.print(channels[i], DEC);   //For reasons that are unclear Channel 16 is not stable. The others seem solid. Signal swings 172 to 1811
    
     if(i==15){
         Serial.print(" \n\r");
     }
     else{
        Serial.print("  ");
      } 
    }
    /*
    for(int i=0; i<16; i++){
      pos[i] = map(channels[0], 172, 1811, 0, 180);
    }

    // tell servo to go to position in variable 'pos' varies from 0 to 180 degrees
    
    emaxservo_01.write(pos[0]);
    emaxservo_02.write(pos[1]);
    emaxservo_03.write(pos[2]);
    emaxservo_04.write(pos[3]);
    emaxservo_05.write(pos[4]);
    emaxservo_06.write(pos[5]);
    emaxservo_07.write(pos[6]);
    emaxservo_08.write(pos[7]);
    emaxservo_09.write(pos[8]);
    emaxservo_10.write(pos[9]);
    emaxservo_11.write(pos[10]);
    emaxservo_12.write(pos[11]);
    emaxservo_13.write(pos[12]);
    emaxservo_14.write(pos[13]);
    emaxservo_15.write(pos[14]);
    emaxservo_16.write(pos[14]);    //Due to protocol instability in Channel 16
    */
    //Use Switch SE as it's the only one listed for unknown reasons
    //Goes to plus or minus 100%
    //Away is minus

    if(channels[4] < 200)     //all move together
    {    
      pos[0] = map(channels[0], 172, 1811, 30, 150);      //Tendon
      pos[1] = map(channels[0], 172, 1811, 150, 30);      //Tendon Inverted
      pos[2] = map(channels[1], 172, 1811, 30, 150);      //Outward InterRos
      pos[3] = map(channels[1], 172, 1811, 150, 30);      //Inverted
      pos[4] = map(channels[2], 172, 1811, 30, 150);      //Inward InterRos
      pos[5] = map(channels[2], 172, 1811, 150, 30);      //Inverted

      pos[6] = map(channels[3], 172, 1811, 160, 40);      //Thumb rotation
      
      emaxservo_FTN.write(pos[0]);  // 48 FT N Fore Tendon Non
      emaxservo_MTN.write(pos[0]);  // 49 MT N Middle Tendon Non
      emaxservo_RTI.write(pos[1]);  // 26 RT I Ring Tendon Inverted, 27, 28, 29, 30, 31, 24, 48, 49, 50, 51, 52, 53, 45, 46, 47
      emaxservo_LTI.write(pos[1]);   // 27 LT I Little Tendon Inverted
      
      emaxservo_FON.write(pos[2]);  // 51 FO N Fore Outward Normal
      emaxservo_MON.write(pos[2]);  // 52 MO N Middle Outward Normal
      emaxservo_ROI.write(pos[3]);  // 30 RO I Ring Outward Inverted
      emaxservo_LON.write(pos[2]);  // 28 LO N Little Outward at Knuckle Non-Inverted
      
      emaxservo_FII.write(pos[5]);  // 50 FI I Fore Inward Inverted
      emaxservo_MIN.write(pos[4]);  // 53 MI N Middle Inwards Normal
      emaxservo_RII.write(pos[5]);  // 31 RI I Ring Inward Inverted
      emaxservo_LII.write(pos[5]);  // 29 LI I Little Inward at Knuckle Inverted
      
      emaxservo_TTN.write(pos[0]);  // 46 TT N Thumb Tendon Non
      emaxservo_TON.write(pos[2]); // 47 TO N Thumb Outwards Non
      emaxservo_TII.write(pos[5]);  // 45 TI I Thumb Inwards
      
      emaxservo_TRN.write(pos[6]); // 24 TR N Thumb Rotation Non-Inverted

      pos[7] = map(channels[1], 172, 1811, 0, 1024);      //Wrist
      pos[8] = map(channels[2], 172, 1811, 0, 1024);      //Wrist
      pos[9] = map(channels[3], 172, 1811, 0, 1024);      //Wrist

      Serial.print("Wrist\n\r");
      SERVO.WritePos(1, pos[7], 500);
      SERVO.WritePos(2, pos[8], 500);
      SERVO.WritePos(3, pos[9], 200);
    }
    else      //Turn off wrist
    {
      pos[0] = map(channels[0], 172, 1811, 30, 150);      //Tendon
      pos[1] = map(channels[0], 172, 1811, 150, 30);      //Tendon Inverted
      pos[2] = map(channels[1], 172, 1811, 30, 150);      //Outward InterRos
      pos[3] = map(channels[1], 172, 1811, 150, 30);      //Inverted
      pos[4] = map(channels[2], 172, 1811, 30, 150);      //Inward InterRos
      pos[5] = map(channels[2], 172, 1811, 150, 30);      //Inverted

      pos[6] = map(channels[3], 172, 1811, 160, 40);      //Thumb rotation
      pos[10] = map(channels[0], 172, 1811, 30, 100);      //Tendon
      
      
      emaxservo_FTN.write(pos[0]);  // 48 FT N Fore Tendon Non
      emaxservo_MTN.write(pos[0]);  // 49 MT N Middle Tendon Non
      emaxservo_RTI.write(pos[1]);  // 26 RT I Ring Tendon Inverted, 27, 28, 29, 30, 31, 24, 48, 49, 50, 51, 52, 53, 45, 46, 47
      emaxservo_LTI.write(pos[1]);   // 27 LT I Little Tendon Inverted
      
      emaxservo_FON.write(pos[5]);  // 51 FO N Fore Outwards Normal
      emaxservo_MON.write(pos[2]);  // 52 MO N Middle Outwards Normal
      emaxservo_ROI.write(pos[3]);  // 30 RO I Ring Outward Inverted
      emaxservo_LON.write(pos[2]);  // 28 LO N Little Outward at Knuckle Non-Inverted
      
      emaxservo_FII.write(pos[2]);  // 50 FI I Fore Inward Inverted
      emaxservo_MIN.write(pos[4]);  // 53 MI N Middle Inwards Normal
      emaxservo_RII.write(pos[5]);  // 31 RI Ring Inward
      emaxservo_LII.write(pos[5]);  // 29 LI I Little Inward at Knuckle Inverted
      
      emaxservo_TTN.write(pos[10]);  // 46 TT N Thumb Tendon Non
      emaxservo_TON.write(pos[2]); // 47 TO N Thumb Outwards Non
      emaxservo_TII.write(pos[5]);  // 45 TI I Thumb Inwards

      emaxservo_TRN.write(pos[6]); // 24 TR N Thumb Rotation Non-Inverted   
    }

   }

  else if (Serial3.available())       //Xbee on Serial3
  {
    default_counter = 1;              //Starts counter in event no Xbee signal received
    
    int inByte = Serial3.read();          //read in the byte

    if(bytecount >= 0 )        bytecount++;                          //Increment once per loop

    //Debug to terminal
    //Serial.print("\n\r");
    //Serial.print("IN ");
    //Serial.print(inByte, DEC);
    //Serial.print(", ");
    //Serial.print(" BC ");
    //Serial.print(bytecount, DEC);
    //Serial.print("\n\r");
    //Serial.print(" ");         
  
    //read and decode the serial bus data cmoing over the xbee
    if(inByte == 0xFE && bytecount == 1)      //bytecount == 1, i.e. byte 1 - First 0xFE toggle flag
    {
        bus_start = 1;
        //Serial.print("bus start \n\r");
        //Serial.print(bytecount, DEC);
        //Serial.print("    ");
        
    }
    else if(inByte == 0xFE && bytecount == -1)      //bytecount == -1, bus had to force restart
    {
        bus_start = 1;
        bytecount = 1;
        Serial.print("bus force restart ");            
    }
    else if(inByte == 0xFE && bytecount == 2 && bus_start == 1)   //byte 2 - Two 0xFE received in a row - should not be possible to generate these angles
    {
        bus_start = 2;
        //Serial.print("Start message   ");
    }
    else if(bytecount == 3 && bus_start == 2)             //3 First byte of data after start bytes so initialise
    {
        //Serial.print(inByte);
        //Serial.print("    ");
        //Serial.println();
        angles[bytecount - 3] = inByte;
        checksum = inByte;
    }
    
    else if(bytecount >= 4 && bytecount <26) //load angles straight in
    {
        angles[bytecount - 3] = inByte;
        //Serial.print(bytecount);
        checksum += inByte;
        
    }
    else if(bytecount == 26)             //byte  - CheckSum
    {
        checksum += inByte;
        //Serial.print(checksum, BIN);
        lo = lowByte(checksum);
        //Serial.print("bc %d  ", bytecount);
        //Serial.print(" Check   ");
        //Serial.print(lo);
        //Serial.print(" \n\r");
    }
    else if(inByte == 0xFD && bytecount == 27 && lo == 255)             //byte 13 - Toggle end of bus flag
    {
        bus_end = 1;
        //Serial.print("End byte 1");

    }
            
    else if(inByte == 0xFD && bytecount == 28 && bus_end == 1)             //byte 14 - Resets and toggle command received flag everything good
    {
        //reset bus
        bytecount = 0;
        checksum = 0;
        lo = 0;
        bus_end = 0;
        bus_start = 0;

        //Trigger servos. Each action must only issue once.
        action = 1;
        
        //Serial.print("Message Received correctly \n\r");
        for(a=0; a<23; a++)
        {
          Serial.print(angles[a]);
          Serial.print("  ");
        }
        Serial.print("\n\r");
        
        
        //Serial.print(action);
        //Serial.print(" \n\r");
    }
        
    else if(bytecount > 28)                                                 //End of message missed error
    {
        bytecount = -1;                                               //reset bytecount, etc so the cycle runs again when first byte 0xAA received
        checksum = 0;
        lo = 0;
        digitemp = 0;
        bus_end = 0;
        bus_start = 0;
        action = 0;

        Serial.print("Serial bus error\n\r");
    }       

    if(action == 1)
    {
      action = 0;                   //Only trigger once
  
      //emaxservo_FON.attach(50);  // 51 FO N Fore Outwards Normal
      //emaxservo_FII.attach(51);  // 50 FI I Fore Inward Inverted
      //emaxservo_FTN.attach(48);  // 48 FT N Fore Tendon Non
       
      //emaxservo_MON.attach(52);  // 52 MO N Middle Outwards Normal
      //emaxservo_MIN.attach(53);  // 53 MI N Middle Inwards Normal
      //emaxservo_MTN.attach(49);  // 49 MT N Middle Tendon Non
      
      //emaxservo_ROI.attach(30);  // 30 RO I Ring Outward Inverted
      //emaxservo_RII.attach(31);  // 31 RI I Ring Inward Inverted 
      //emaxservo_RTI.attach(26);  // 26 RT I Ring Tendon Inverted
      
      //emaxservo_LON.attach(28);  // 28 LO N Little Outward at Knuckle Non-Inverted
      //emaxservo_LII.attach(29);  // 29 LI I Little Inward at Knuckle Inverted
      //emaxservo_LTI.attach(27);  // 27 LT I Little Tendon Inverted
      
      //emaxservo_TON.attach(47);  // 47 TO N Thumb Outwards Non
      //emaxservo_TII.attach(45);  // 45 TI I Thumb Inwards Inverted
      //emaxservo_TTN.attach(46);  // 46 TT N Thumb Tendon Non
      //emaxservo_TRN.attach(24);  // 24 TR N Thumb Rotation Non-Inverted
  
      //First finger: knuckle flex angles[0] - range 150 (50 previously) to 170
      //              knuckle yaw angles[1] - range 80 (75) to 100 (105)
      //              long tendon angles[2] - range 150 (25 previously) to 175
  
      //angles[0] = constrain(angles[0], 50, 170); 
      angles[0] = constrain(angles[0], 150, 170);
      //angles[1] = constrain(angles[1], 75, 105);
      angles[1] = constrain(angles[1], 80, 100);
      //angles[2] = constrain(angles[2], 25, 175);
      angles[2] = constrain(angles[2], 150, 175);
  
      // Compensate for model error
      //if((93 - angles[1]) > 0)
        //angles[0] = angles[0] + 1.5*(93 - angles[1]);
      
      servo_pos[0] = angles[0] + ((93 - angles[1])/2);                  //FON small number is tight
      servo_pos[0] = map(servo_pos[0], 143.5, 173.5, 30, 150);  
      
      servo_pos[1] = angles[0] - ((93 - angles[1])/2);          //FII large numbre is loose
      servo_pos[1] = map(servo_pos[1], 173.5, 143.5, 30, 150);
      
      servo_pos[2] = angles[2];                               //FTN
      servo_pos[2] = map(servo_pos[2], 175, 150, 30, 150);
  
      emaxservo_FON.write(servo_pos[0]);
      emaxservo_FII.write(servo_pos[1]);
      emaxservo_FTN.write(servo_pos[2]);
      /*
      Serial.print(angles[0]);
      Serial.print(" ");
      Serial.print(angles[1]);
      Serial.print(" ");
      Serial.print(angles[2]);
      Serial.print(" ");
      Serial.print(servo_pos[0]);
      Serial.print(" ");
      Serial.print(servo_pos[1]);
      Serial.print(" ");
      Serial.print(servo_pos[2]);
      Serial.print("\n\r");
      */
  
      //Middle finger: knuckle flex angles[3] - range 50 to 170
      //              knuckle yaw angles[4] - range 75 to 105
      //              long tendon angles[5] - range 25 to 175
  
      angles[3] = constrain(angles[3], 150, 175);
      angles[4] = constrain(angles[4], 80, 100);
      angles[5] = constrain(angles[5], 150, 175);
  
      servo_pos[3] = angles[3] - ((100 - angles[4])/2);                  //MON small number is tight
      servo_pos[3] = map(servo_pos[3], 175, 140, 30, 150);  
      
      servo_pos[4] = angles[3] + ((100 - angles[4])/2);          //MIN large numbre is loose
      servo_pos[4] = map(servo_pos[4], 175, 140, 30, 150);
      
      servo_pos[5] = angles[5];                               //MTN
      servo_pos[5] = map(servo_pos[5], 175, 150, 30, 150);
  
      emaxservo_MON.write(servo_pos[3]);
      emaxservo_MIN.write(servo_pos[4]);
      emaxservo_MTN.write(servo_pos[5]);
      
      //emaxservo_MON.attach(52);  // 52 MO N Middle Outwards Normal
      //emaxservo_MIN.attach(53);  // 53 MI N Middle Inwards Normal
      //emaxservo_MTN.attach(49);  // 49 MT N Middle Tendon Non    
  /*
      Serial.print(angles[3]);
      Serial.print(" ");
      Serial.print(angles[4]);
      Serial.print(" ");
      Serial.print(angles[5]);
      Serial.print(" ");
      Serial.print(servo_pos[3]);
      Serial.print(" ");
      Serial.print(servo_pos[4]);
      Serial.print(" ");
      Serial.print(servo_pos[5]);
      Serial.print("\n\r");
  */
      
      //Ring finger: knuckle flex angles[0] - range 50 to 170
      //              knuckle yaw angles[1] - range 75 to 105
      //              long tendon angles[2] - range 25 to 175
  
      angles[6] = constrain(angles[6], 150, 170);
      angles[7] = constrain(angles[7], 80, 100);
      angles[8] = constrain(angles[8], 150, 175);

      ring_offset = 10;
  
      servo_pos[6] = angles[6] + ((90 - angles[7])/2);                  //ROI
      servo_pos[6] = map(servo_pos[6], 145, 175, 30, 150);  
      servo_pos[6] = servo_pos[6] - ring_offset;
      
      servo_pos[7] = angles[6] - ((90 - angles[7])/2);          //RII
      servo_pos[7] = map(servo_pos[7], 145, 175, 30, 150);
      servo_pos[7] = servo_pos[7] + ring_offset;
      
      servo_pos[8] = angles[8];                               //RTI
      servo_pos[8] = map(servo_pos[8], 150, 175, 30, 150);
  
      emaxservo_ROI.write(servo_pos[6]);
      emaxservo_RII.write(servo_pos[7]);
      emaxservo_RTI.write(servo_pos[8]);
      
      //emaxservo_ROI.attach(30);  // 30 RO I Ring Outward Inverted
      //emaxservo_RII.attach(31);  // 31 RI I Ring Inward Inverted 
      //emaxservo_RTI.attach(26);  // 26 RT I Ring Tendon Inverted
  
      //Little finger: knuckle flex angles[0] - range 50 to 170
      //              knuckle yaw angles[1] - range 75 to 105
      //              long tendon angles[2] - range 25 to 175
  
      angles[9] = constrain(angles[9], 150, 170);
      angles[10] = constrain(angles[10], 80, 100);
      angles[11] = constrain(angles[11], 150, 175);
  
      servo_pos[9] = angles[9] + ((90 - angles[10])/2);                  //LON
      servo_pos[9] = map(servo_pos[9], 175, 145, 30, 150);  
      
      servo_pos[10] = angles[9] - ((90 - angles[10])/2);                  //LII
      servo_pos[10] = map(servo_pos[10], 145, 175, 30, 150);
      
      servo_pos[11] = angles[11];                                     //LTI
      servo_pos[11] = map(servo_pos[11], 150, 175, 30, 150);
  
      emaxservo_LON.write(servo_pos[9]);
      emaxservo_LII.write(servo_pos[10]);
      emaxservo_LTI.write(servo_pos[11]);
      
      //emaxservo_LON.attach(28);  // 28 LO N Little Outward at Knuckle Non-Inverted
      //emaxservo_LII.attach(29);  // 29 LI I Little Inward at Knuckle Inverted
      //emaxservo_LTI.attach(27);  // 27 LT I Little Tendon Inverted
      //emaxservo_TRN.attach(24);  // 24 TR N Thumb Rotation Non-Inverted Range 110 135
      
      //Thumb: knuckle flex angles[12] - range 150 to 170
      //              knuckle yaw angles[13] - range 30 to 60
      //              long tendon angles[14] - range 150 to 170
      //              basal rotation angles[15]
      
      angles[12] = constrain(angles[12], 150, 170);
      angles[13] = constrain(angles[13], 30, 60);
      angles[14] = constrain(angles[14], 150, 170);
      angles[15] = constrain(angles[15], 110, 135);
      
      servo_pos[12] = angles[12] + ((45 - angles[13])/2);                  //TON
      servo_pos[12] = map(servo_pos[12], 177.5, 142.5, 30, 150);  
      
      servo_pos[13] = angles[12] - ((45 - angles[13])/2);                  //TII
      servo_pos[13] = map(servo_pos[13], 142.5, 177.5, 30, 150);
      
      servo_pos[14] = angles[14];                                     //TIN
      servo_pos[14] = map(servo_pos[14], 170, 150, 30, 150);
      
      servo_pos[15] = angles[15];                                     //TRN
      servo_pos[15] = map(servo_pos[15], 110, 135, 100, 150);
      
      emaxservo_TON.write(servo_pos[12]);
      emaxservo_TII.write(servo_pos[13]);
      emaxservo_TTN.write(servo_pos[14]);
      emaxservo_TRN.write(servo_pos[15]);
      
      //emaxservo_TON.attach(47);  // 47 TO N Thumb Outwards Non
      //emaxservo_TII.attach(45);  // 45 TI I Thumb Inwards Inverted
      //emaxservo_TTN.attach(46);  // 46 TT N Thumb Tendon Non
      //emaxservo_TRN.attach(24);  // 24 TR N Thumb Rotation Non-Inverted
      /*
      Serial.print(angles[12]);
      Serial.print(" ");
      Serial.print(angles[13]);
      Serial.print(" ");
      Serial.print(angles[14]);
      Serial.print(" ");
      Serial.print(angles[15]);
      Serial.print(" ");
      Serial.print(servo_pos[12]);
      Serial.print(" ");
      Serial.print(servo_pos[13]);
      Serial.print(" ");
      Serial.print(servo_pos[14]);
      Serial.print(" ");
      Serial.print(servo_pos[15]);
      Serial.print("\n\r");      

      // angles[16] is wrist pitch, range 70 to 110, centre 90
      // angles[17] is wrist yaw, range 25 to 65, centre 45
      // angles[18] is wrist rotation around long axis of forearm, range 70 to 110, centre 90
      */
      
      angles[16] = constrain(angles[16], 70, 110);
      angles[17] = constrain(angles[17], 25, 65);
      angles[18] = constrain(angles[18], 70, 115);

      
      //Wrist is sum and difference of two angles and z rotation coincident with long axis of forearm
      wrist_pos[0] = map(((angles[16] - 90) - (angles[17] - 45)), 40, -40, 100, 724);     // wrist pitch
      wrist_pos[1] = map(((angles[16] - 90) + (angles[17] - 45)), -40, 40, 300, 924);     // wrist yaw
      wrist_pos[2] = map(angles[18], 70, 115, 400, 800);                                  // wrist rotation

      Serial.print("Wrist\n\r");
      SERVO.WritePos(1, wrist_pos[0], 200, 400);               // bottom outer
      SERVO.WritePos(2, wrist_pos[1], 200, 400);               // top outer
      SERVO.WritePos(3, wrist_pos[2], 50, 200);               // 3 wrist rotation

      Serial.print(angles[16]);
      Serial.print(" ");
      Serial.print(angles[17]);
      Serial.print(" ");
      Serial.print(angles[18]);
      Serial.print(" ");
      Serial.print(wrist_pos[0]);
      Serial.print(" ");
      Serial.print(wrist_pos[1]);
      Serial.print(" ");
      Serial.print(wrist_pos[2]);
      Serial.print("\n\r");      

    }

    default_counter = 1;
    default_stage = 0;
  } //If Xbee receive data
  
  else
  {
    if(default_counter >= 1)
    {
      default_counter++;
  
      if(default_counter == 15000 && default_stage == 0)
      { 
        default_stage = 1;
      }
      else if(default_counter % 100000 == 0 && default_stage >= 2)
      {
        default_stage++;
      }

      if(default_stage == 1)
      { 
        emaxservo_FON.write(140);  // 51 FO N Fore Outwards Normal
        emaxservo_FII.write(40);  // 50 FI I Fore Inward Inverted
        emaxservo_FTN.write(40);  // 48 FT N Fore Tendon Non
         
        emaxservo_MON.write(40);  // 52 MO N Middle Outwards Normal
        emaxservo_MIN.write(40);  // 53 MI N Middle Inwards Normal
        emaxservo_MTN.write(40);  // 49 MT N Middle Tendon Non
        
        emaxservo_ROI.write(140);  // 30 RO I Ring Outward Inverted
        emaxservo_RII.write(140);  // 31 RI I Ring Inward Inverted 
        emaxservo_RTI.write(140);  // 26 RT I Ring Tendon Inverted
        
        emaxservo_LON.write(50);  // 28 LO N Little Outward at Knuckle Non-Inverted
        emaxservo_LII.write(140);  // 29 LI I Little Inward at Knuckle Inverted
        emaxservo_LTI.write(140);  // 27 LT I Little Tendon Inverted
        
        emaxservo_TON.write(40);  // 47 TO N Thumb Outwards Non 150 long
        emaxservo_TII.write(40);  // 45 TI I Thumb Inwards Inverted 150 long
        emaxservo_TTN.write(40);  // 46 TT N Thumb Tendon Non 30 long
        emaxservo_TRN.write(120);  // 24 TR N Thumb Rotation Non-Inverted 150 inwards

        //Wrist is sum and difference of two angles and z rotation coincident with long axis of forearm
        SERVO.WritePos(1, 350, 100, 200);               // 1 bottom outer - lower pitch back
        SERVO.WritePos(2, 650, 100, 200);               // 2 top outer -higher pitch back
        SERVO.WritePos(3, 580, 500, 100);               // 3 wrist rotation

        default_stage = 2;
        //default_counter = 0;
     }
     else if(default_stage == 3)     // Skip 2
     {
        emaxservo_FON.write(40);  // 51 FO N Fore Outwards Normal
        emaxservo_FII.write(140);  // 50 FI I Fore Inward Inverted
        emaxservo_FTN.write(140);  // 48 FT N Fore Tendon Non        
     }
     else if(default_stage == 4)     // Skip 2
     {
        emaxservo_FON.write(140);  // 51 FO N Fore Outwards Normal
        emaxservo_FII.write(40);  // 50 FI I Fore Inward Inverted
        emaxservo_FTN.write(40);  // 48 FT N Fore Tendon Non
        
        emaxservo_MON.write(140);  // 52 MO N Middle Outwards Normal
        emaxservo_MIN.write(140);  // 53 MI N Middle Inwards Normal
        emaxservo_MTN.write(140);  // 49 MT N Middle Tendon Non   
     }
     else if(default_stage == 5)
     {  
        emaxservo_MON.write(40);  // 52 MO N Middle Outwards Normal
        emaxservo_MIN.write(40);  // 53 MI N Middle Inwards Normal
        emaxservo_MTN.write(40);  // 49 MT N Middle Tendon Non
        
        emaxservo_ROI.write(40);  // 30 RO I Ring Outward Inverted
        emaxservo_RII.write(40);  // 31 RI I Ring Inward Inverted 
        emaxservo_RTI.write(40);  // 26 RT I Ring Tendon Inverted        
     }
     else if(default_stage == 6)
     {
        emaxservo_ROI.write(140);  // 30 RO I Ring Outward Inverted
        emaxservo_RII.write(140);  // 31 RI I Ring Inward Inverted 
        emaxservo_RTI.write(140);  // 26 RT I Ring Tendon Inverted
        
        emaxservo_LON.write(140);  // 28 LO N Little Outward at Knuckle Non-Inverted
        emaxservo_LII.write(40);  // 29 LI I Little Inward at Knuckle Inverted
        emaxservo_LTI.write(40);  // 27 LT I Little Tendon Inverted        
     }
     else if(default_stage == 7)
     {
        emaxservo_LON.write(50);  // 28 LO N Little Outward at Knuckle Non-Inverted
        emaxservo_LII.write(140);  // 29 LI I Little Inward at Knuckle Inverted
        emaxservo_LTI.write(140);  // 27 LT I Little Tendon Inverted
        
        emaxservo_TON.write(140);  // 47 TO N Thumb Outwards Non 150 long
        emaxservo_TII.write(140);  // 45 TI I Thumb Inwards Inverted 150 long
        emaxservo_TTN.write(140);  // 46 TT N Thumb Tendon Non 30 long
        emaxservo_TRN.write(140);  // 24 TR N Thumb Rotation Non-Inverted 150 inwards                   
     }
     else if(default_stage == 8)
     {
        emaxservo_TON.write(40);  // 47 TO N Thumb Outwards Non 150 long
        emaxservo_TII.write(40);  // 45 TI I Thumb Inwards Inverted 150 long
        emaxservo_TTN.write(40);  // 46 TT N Thumb Tendon Non 30 long
        emaxservo_TRN.write(120);  // 24 TR N Thumb Rotation Non-Inverted 150 inwards      
     }
     else if(default_stage == 9)
     {
        default_counter = 15000;
        default_stage = 0;
     }
     
   }
  }

  
}
