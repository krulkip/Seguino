//---------------------------------------------------------------------------
//                                        _               _ _      
//    __________ _  __ _  __ _  __      _| |__   ___  ___| (_) ___ 
//   |_  /_  / _` |/ _` |/ _` | \ \ /\ / / '_ \ / _ \/ _ \ | |/ _ \
//    / / / / (_| | (_| | (_| |  \ V  V /| | | |  __/  __/ | |  __/
//   /___/___\__,_|\__,_|\__, |   \_/\_/ |_| |_|\___|\___|_|_|\___|
//                       |___/                                     
//                         _             
//    ___  ___  __ _ _   _(_)_ __   ___  
//   / __|/ _ \/ _` | | | | | '_ \ / _ \ 
//   \__ \  __/ (_| | |_| | | | | | (_) |
//   |___/\___|\__, |\__,_|_|_| |_|\___/ 
//             |___/                      
// (c)2010 Krulkip
//---------------------------------------------------------------------------
// Project             : SEGUINO
// Version             : WOF105
// Date                : 24.10.2011
// Comments            : Arduino Segway clone
// Chip type           : ATMega644P
// Clock frequency     : 16 MHz
// Port of the WOF105 software for the Elektor Wheelie
// by Guenter Gerold
// Based on the AT3329 project by Reinhold Pieper
// improved on by Chris Krohne as part of his Elektor firmware FW 2.6
// My hardware is based on his Zzaag design with a few modifications.
// Improvements as per http://www.gerold-online.de/cms/wheelie/faq.html
// These were soldered directly to the PCB's. The 1000uF capacitors were mounted on other side of
// the PCB than normal to create the room to do this. 
// To the PCB print added reset circuit and made RS232 connector suitable for FDTI connector
// SCL and SDA also brought out to this connector to talk to separate gearsensor board.
// These connections coupled to a separate Atmega168 board with ACS755 and gearsensor circuit based on ATS667
// using a single 10 pole bandcable from RS232 connection on ZZaag CPU board.
// These connections are needed to enable the Arduino IDE.
// Some mods were made to the GPIO connector to allow a 6 LED array for battery display etc to be connected.
// I have some pictures and can scan handdrawn circuit diagram if anyone is interested.
// I used the modified core files from http://www.avr-developers.com/ to make the arduino IDE
// suitable for the Atmega644P processor which i used on the Zzaag CPU board
// maybe one day i can put it online. :-)

// To Do:
// Build in filter types eg Kalman DCM etc
// Utilise Arduino PID library
// Easy way to switch on/off 6km/hr limit during standby MMode
// eg by dipswitch on secondary PCB and setting bits in 9th byte & make into MMode
// add watchdog timer
// add X_enable stuff
// add Z correction
// error trapping in parser routine
//---------------------------------------------------------------------------
// includes
#include <avr/interrupt.h>  
#include <avr/io.h> 
#include <EEPROM.h>
#include <I2C.h>

//*************** SWITCHES *************************
//##################################################
#define Zzaag         true                                  // Wheelie = 0 or Zzaag = 1
//##################################################
#define Inverse_poti  true                                  // Inverse steering poti (Zzaag =1) normal steering poti (Wheelie=0)
//##################################################
#define I2C_enable    true                                  // Turn the I2C geartoothsensors on and off
//##################################################
#define X_enable      false                                  // Turn the X-axis on and off
//##################################################
//*************** END  SWITCHES ********************

// definitions

#define Total_pitch_looptime          150        // Looptime for filters
#define Total_pitch_looptime10         15
#define Run                       1
#define Standby                   0
#define Down                      3
#define Sw_down                  50
#define Max_pwm                 180
#define Battdead                732
#define Battlower               737 
#define Battupper               742
#define Battok                  746
#define Gear_Sensor_Address    0x20
#define CONFIG_VERSION "BKa"  // ID of the settings block
#define CONFIG_START 0       // Tell it where to store your config data in EEPROM

// global variables 
   long   Ad_pitch_Accl = 0;  
   long   Ad_pitch_gyro = 0;
   int    Ad_batt = 0;
   int    Ad_swi=0;
   long   Total_pitch_Accl_gyro=0;  
   long   Average_pitch_gyro=0;
   long   Average_batt;
   int    Drivea=0;
   int    Driveb=0;
   long   Buf1=0;
   long   Buf2=0;
   long   Buf=0;
   long   Tilt_pitch_angle=0;
   int    Drive_a=0;
   int    Drive_b=0;
   int    Drivespeed=0;
   int    Steeringsignal=0;
   long   Angle_pitch_rate=0;
   long   Balance_moment=0;
   int    MMode;
   long   Drive_sum=0; 
   long   Ad_steering=0;   
   long   Steeringsq=0;
   long   Steeringold=0;
   long   Steering_zero=0;  
   long   Accl_pitch_zero=0;
   long   Gyro_pitch_zero=0;
   int    Errno=0;  
   long   previousTime = 0;
   boolean testmode = false;
   int Te ;

// Now the limits fopr the parameters
   int Parameter1_max = 30; int Parameter1_min = 8;
   int Parameter2_max = 30; int Parameter2_min = 10;
   int Parameter3_max = 10; int Parameter3_min = - 10;   
   int Parameter1 = 14; int Parameter4 = 10;
   int Parameter2 = 18; int Parameter5 = 15;
   int Parameter3 =  3; int Parameter6 =  1;    
// Storage structure for EEPROM
 struct StoreStruct {
   // This is for mere detection if they are your settings
   char version[4];
   // The variables of your settings
   int Parameter1 ;   // speed-sensitive steering response: higher value = less sensitive steering
   int Parameter2 ;   // Influence of the speed difference of the wheels to the steering: higher value = less influence
   int Parameter3 ;
   int Parameter4 ;
   int Parameter5 ;
   int Parameter6 ;
 } storage = {
   CONFIG_VERSION,
   // The default values
   Parameter1, Parameter2, Parameter3, Parameter4, Parameter5, Parameter6
 };

   // My gearbox has 60 teeth. Yes i counted. My wheel circumference is 126cm. Circumference = Pi * Diameter
   // Gearsensor sends ticks between teeth. This is 1024 / 16.000.000 seconds per tick because prescaler set at 1024
   // and 16.000.000 is clock speed. There are 3600 seconds in an hour and 100.000 cm in a Km
   // This means speed = circumference / ticks * 16.000.000 * 3600 / 60 / 1024 / 100.000
   // speed = circumference / ticks * 9.37476. We use speed * 10 to avoid rounding mistakes
   // Means Wheel_const = 93.37476 * 126 = 11812
   long Wheel_const  = 11812;
   int No_speed_limit_flag     =  1;
#if Zzaag
//**************** ZZAAG HARDWARE *******************
   #define Ad_channel_foot                   4
   #define Ad_channel_batt                   6
   #define Ad_channel_steering               5
   #define Ad_channel_pitch_Accl             1 
   #define Ad_channel_pitch_gyro             3
   #define Ad_channel_roll_Accl              0 
   #define Ad_channel_roll_gyro              2

   int Cw_ccw_a   = 15;   // Portd.7
   int Cw_ccw_b   = 14;   // Portd.6
   int PWM_a      = 13;   // Portd.5
   int PWM_b      = 12;   // Portd.4
   int Buzzer     = 11;   // Portd.3
   int Power_off  = 10;   // Portd.2  (Error Portc.2 WOF105 for wheelie should be Portd.2) (Wheelie & Wheelie II is Portd.3)   
   int LED1       = 23;   // Portc.2  battery status
   int LED2       = 18;   // Portc.3  battery status
   int LED3       = 20;   // Portc.4  battery status
   int LED4       = 22;   // Portc.5  battery status
   int LED5       = 19;   // Portc.6  heartbeat
   int LED6       = 21;   // Portc.7  power
   int pinB0      =  1;   // Portb.0  check
   int pinB1      =  2;   // PortB.1  check
//**************** END ZZAAG HARDWARE *******************
#else
//**************** WHEELIE HARDWARE *******************
// I dont have a wheelie so can not test. Add hardware defines here
//**************** END WHEELIE HARDWARE *******************
#endif
/*----------------------------------------------------------------------------  
I2C  part
-----------------------------------------------------------------------------*/
   #if I2C_enable
   uint8_t    Speed_array[9] = {0,0,0,0,0,0,0,0,0};
   uint16_t* Speed_array16 = (uint16_t*)Speed_array;
   #define    Speed_left  Speed_array16[0]
   #define    Speed_right Speed_array16[1]
   #define    Curr_left   Speed_array16[2]
   #define    Curr_right  Speed_array16[3]
   int    Speed_left_out = 0;
   int    Speed_right_out = 0;
   int    Speed_sum=0;   
   int    Speed_diff;
   int    Speed_correction=0; 
   int    Speed_error=0;
   int    Steering_position=0;
   int    Steering_sensitivity=0;
   long   Temp_Accl_pitch_zero;
   #endif
/*----------------------------------------------------------------------------  
End I2C part
-----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------  
X-Axis  part
-----------------------------------------------------------------------------*/
   #if X_enable
   long Ad_roll_Accl;
   long Ad_roll_gyro;
   long Total_roll_Accl_gyro;
   long Average_roll_gyro;
   long Tilt_roll_angle = 0;
   long Angle_roll_rate;
   long Buf_roll;
   long Accl_roll_zero;
   long Gyro_roll_zero;
   int Total_roll_looptime = 20;
   int Total_roll_looptime10 = 2;
   #endif
/*----------------------------------------------------------------------------  
End X-Axis part
-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------  
Telemetry part
-----------------------------------------------------------------------------*/
   String Identity = "WOF105";
   char *ptr;
   int Channel;
   int Value;
   int NextChar;
   int StringCounter = 0;
   String Inputstring;
   String Parsing;
   String Statustext;
/*----------------------------------------------------------------------------  
End Telemtry part
-----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------
Function           : setup()
Initialise the SEGUINO Hardware
Date               : 25.09.2010
-----------------------------------------------------------------------------*/
void setup()
{
    Serial.begin(57600);
    Serial.flush();
    I2c.begin();
    analogReference(EXTERNAL);    
    // To make sure there are settings, and they are YOURS!
    // If nothing is found it will use the default settings.
    if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
       EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
       EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
    for (unsigned int t=0; t<sizeof(storage); t++) {
       *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
     }
    if (storage.Parameter1 > Parameter1_max){ storage.Parameter1 =  Parameter1_max;} 
    if (storage.Parameter1 < Parameter1_min){ storage.Parameter1 =  Parameter1_min;}
    if (storage.Parameter2 > Parameter2_max){ storage.Parameter2 =  Parameter2_max;}
    if (storage.Parameter2 < Parameter2_min){ storage.Parameter2 =  Parameter2_min;}
    if (storage.Parameter3 > Parameter3_max){ storage.Parameter3 =  Parameter3_max;}
    if (storage.Parameter3 < Parameter3_max){ storage.Parameter3 =  Parameter3_min;}    
    
    PORTA = 0x00;
    DDRA  = 0x00;// Data Direction : a 1 = output here no pullups
    PORTB = 0xE2;// X, X, X, X, X, MOSI,MISO,SCK
    DDRB  = 0xE3;
pinMode (Power_off,OUTPUT);   
pinMode (Cw_ccw_a,OUTPUT);
pinMode (Cw_ccw_b,OUTPUT);
pinMode (Buzzer,OUTPUT);
pinMode (PWM_a,OUTPUT);
pinMode (PWM_b,OUTPUT);
pinMode (LED1,OUTPUT);
pinMode (LED2,OUTPUT);
pinMode (LED3,OUTPUT);
pinMode (LED4,OUTPUT);
pinMode (LED5,OUTPUT);
pinMode (LED6,OUTPUT);
pinMode (pinB0,INPUT);
pinMode (pinB1,INPUT);
int DB[] = {LED1,LED2,LED3,LED4,LED5,LED6};

Average_batt = Battok;
MMode = Standby;
  for (int i=0;i<2;i++){
    for(int j=0;j<6;j++){ digitalWrite(DB[j], HIGH);delay(50); }
    for(int j=0;j<6;j++){ digitalWrite(DB[j], LOW);delay(50); }
  }  
  for (int i=0;i<2;i++){
    for(int j=0;j<6;j++){ digitalWrite(DB[j], HIGH);delay(50); }
    for(int j=0;j<6;j++){ digitalWrite(DB[5-j], LOW);delay(50); }
  } 
  for (int i=0;i<2;i++){
    for(int j=0;j<6;j++){ digitalWrite(DB[j], HIGH);delay(50);digitalWrite(DB[j], LOW);delay(50); }
  }  
Signal();Signal();// biep biep
Ad_swi = 0;
for (Buf=1;Buf<11;Buf++) {
  Ad_swi = Ad_swi + analogRead(Ad_channel_foot);
  delay(1);
}
Ad_swi = Ad_swi/10;
if (Ad_swi < 100) { Errno = 4; Serial.print ("2:11=Footswitch pressed"); Err_value(); }
Statustext = Identity;
    digitalWrite(Power_off,LOW);// is reverse LOW means Power on
    digitalWrite(LED6,HIGH);//show power on
    TCCR1A = 1<<COM1A1 | 0<<COM1A0 | 1<<COM1B1 | 1<<COM1B0 | 0<<WGM11 | 1<<WGM10;           // COM1A1 COM1A0 COM1B1 COM1B0 X X WGM11 WGM10 --> 1011 0001 = 0xB1
    TCCR1B = 0<<ICNC1 | 0<<ICES1 | 0<<WGM13 | 0<<WGM12 | 0<<CS12 | 0<<CS11 | 1<<CS10;       // FOC1A FOC1B WGM12 CS12 CS11 CS10            --> 0000 0001 = 0x01
    TCCR1C = 0;
    // CS12 CS11 CS10 001 --> No prescalar  :  WGM13 WGM12 WGM11 WGM10 0001 --> Phase correct PWM 8 bits TOP=0xFF   --> 16MHz/2/256 = 31.250 Hz
    // In AT2239 project PWM frequency was 15.625 Hz  *********************************************  To Do : Check impact of this 
    TCCR2A = 0<<COM2A1 | 0<<COM2A0 | 0<<COM2B1 | 0<<COM2B0 | 1<<WGM21 | 0<<WGM20;// COM2A1 COM2A0 COM2B1 COM2B0 X X WGM21 WGM20 --> 0000 0010 = 0x02
    TCCR2B = 0<<FOC2A | 0<<FOC2B | 0<<WGM22 | 1<<CS22 | 1<<CS21 | 1<<CS20;       // FOC2A FOC2B WGM22 CS22 CS21 CS20            --> 0000 0101 = 0x05
    OCR2A = 156;
    // CLK = 16.000.000 MHz  :  CS22 CS21 CS20 = 111 --> prescalar 1024  --> 15.625kHz  :  156 counts --> 9.984ms
    // WGM21 --> mode 2 CTC clear timer on compare match
    TCNT2 = 0;
    OCR1A = 255;
    OCR1BL = 0; 
    TIMSK2 = 0<<OCIE2B | 0<<TOIE2 | 0<<OCIE2A;// enable compare interrupt 
    sei();
previousTime = millis();  // mark the initial time
}

/*-----------------------------------------------------------------------------
Function           : loop()
 main loop
Date               : 07.08.2010
-----------------------------------------------------------------------------*/
void loop()
{
  if((millis() - previousTime) >= 1000 ) {    // if one second has passed since last mark
     previousTime = millis();  // mark again
     Buf = digitalRead(LED5) ^ 1;//     Do an activity eg Heartbeat LED
     digitalWrite(LED5,Buf) ;
  }
  if (testmode == false) {
  if (MMode != Standby) {    
  #if X_enable
  Serial.print ("2:1=");Serial.print(Tilt_roll_angle);Serial.print("\t");delay(8);         //angle sideways
  #endif
  Serial.print ("2:2=");Serial.print(Tilt_pitch_angle);Serial.print("\t");delay(8);           //tilt angle
  #if I2C_enable
  Serial.print ("2:3=");Serial.print(Steeringsq); Serial.print("\t");delay(8);            //steering position
  Serial.print ("2:4=");Serial.print(Steering_sensitivity); Serial.print("\t");delay(8);  //steering sensitivity  
  Serial.print ("2:5=");Serial.print(Speed_error);Serial.print("\t");delay(8);          //speederror
  Serial.print ("2:9=");Serial.print(Speed_left_out);Serial.print("\t");delay(8);       //speed * 10 Left wheel
  Serial.print ("2:10=");Serial.print(Speed_right_out);Serial.print("\t");delay(8);     //speed * 10 Right wheel
  Serial.print ("2:15=");Serial.print(Curr_left);Serial.print("\t");delay(8);           //current left 0 - 1024
  Serial.print ("2:16=");Serial.print(Curr_right);Serial.print("\t");delay(8);          //current right 0 - 1024
  #endif
  Serial.print ("2:6=");Serial.print(Steeringsignal);Serial.print("\t");delay(8);       //steeringsignal
  Serial.print ("2:7=");Serial.print(Drive_a);Serial.print("\t");delay(8);              //power left -250 to 250
  Serial.print ("2:8=");Serial.print(Drive_b);Serial.print("\t");delay(8);              //power right -250 to 250
//  Serial.print ("2:13=");Serial.print("512");Serial.print("\t");
//  Serial.print ("2:14=");Serial.print("512");Serial.print("\t");
  }
  else {
  Serial.print ("2:17=");Serial.print(storage.Parameter1);Serial.print("\t");delay(8);
  Serial.print ("2:18=");Serial.print(storage.Parameter2);Serial.print("\t");delay(8);
  Serial.print ("2:19=");Serial.print(storage.Parameter3);Serial.print("\t");delay(8);
  Serial.print ("2:20=");Serial.print(storage.Parameter4);Serial.print("\t");delay(8);
  Serial.print ("2:21=");Serial.print(storage.Parameter5);Serial.print("\t");delay(8);
  Serial.print ("2:22=");Serial.print(storage.Parameter6);Serial.print("\t");delay(8);
  Parser();
  }
  Serial.print ("2:12=");Serial.print(Ad_batt); Serial.print("\t");delay(8);             // Battery 732 = 24Volt 746= 25Volt
  Serial.print ("2:11=");Serial.print(Statustext);Serial.print("\r\n");delay(8);
  Process();
  }
  else { 
        Drive_a = 0; Drive_b = 0;
        Set_pwm();
        TCCR2A &= ~(1<<WGM21);                        // stop timer 2
        TCCR2B &= ~(1<<CS20) | (1<<CS21) | (1<<CS22); // stop timer 2
        TIMSK2 &= ~(1<<OCIE2A);                       // disable compare interrupt
       digitalWrite(LED1,LOW);digitalWrite(LED2,LOW);digitalWrite(LED3,LOW);
       digitalWrite(LED4,LOW);digitalWrite(LED5,LOW);digitalWrite(LED6,LOW);
       Serial.println ("Wheelie / Zzaag Testprogram ");
       Serial.println ("Start -> Confirm by pressing footswitch ");
       Te = 1;  while (Te > 0) { F_switch(); }
       // --------- Motoren testen ----------------------------      
       digitalWrite(LED1,HIGH);Serial.println ("right wheel forward");
       Drive_a = 50; Drive_b = 0; Set_pwm(); Te = 1; while (Te > 0) { F_switch(); }     //Right wheel forward
       digitalWrite(LED2,HIGH);Serial.println ("left wheel forward");
       Drive_a = 0; Drive_b = 50; Set_pwm(); Te = 1; while (Te > 0) { F_switch();}      //Left wheel forward
       digitalWrite(LED3,HIGH);Serial.println ("right wheel backwards");
       Drive_a = -50;Drive_b = 0; Set_pwm(); Te = 1; while (Te > 0) { F_switch();}     //Right wheel backwards
       digitalWrite(LED4,HIGH);Serial.println ("left wheel backwards");
       Drive_a = 0; Drive_b = -50; Set_pwm(); Te = 1; while (Te > 0) { F_switch();}     //Left wheel backwards
       Drive_a = 0; Drive_b = 0;  Set_pwm(); Te = 1;
       Serial.println ("Test steering --> Steering position gives speed of wheels ");
       Serial.println ("Steer wheelie to see impact on wheels !!! ");
       digitalWrite(LED5,HIGH);
// --------- test steering ----------------------------
   //Calculate mid position of steering (Steering_zero)
   Ad_steering = 0;
   for (Buf=1;Buf<11;Buf++) {                                        //Mean out of 10 values
      #if !Inverse_poti
      Ad_steering += analogRead(Ad_channel_steering);
      #else
      Ad_steering = Ad_steering + 1024 - analogRead(Ad_channel_steering);
      #endif
      delay(1);
   }
   Steering_zero = Ad_steering / 10;
         while (Te > 0) {      
           #if !Inverse_poti                    // added BK 23/4/2011 for correctness although not strictly needed
           Ad_steering = analogRead(Ad_channel_steering);
           #else
           Ad_steering = 1024 - analogRead(Ad_channel_steering);
           #endif
           Steeringsq = Steering_zero - Ad_steering;
           if (Steeringsq > 50) { Steeringsq = 50;} if (Steeringsq < -50) {Steeringsq = -50;}
           Drive_a = Steeringsq; Drive_b = Steeringsq * -1; Set_pwm(); delay(1); F_switch(); delay(10);
         }
         Drive_a = 0; Drive_b = 0; Set_pwm(); digitalWrite(LED6,HIGH);
         // --------- Fetch Pitch zero ----------------------------
         Serial.println ("Test pitch --> slope of wheelie sets pace of both wheels"); 
         Serial.println ("Tilt wheelie to see impact on wheels !!!");Accl_pitch_zero = 0;
         for (Buf = 1; Buf < 11; Buf++) { Ad_pitch_Accl = analogRead(Ad_channel_pitch_Accl); Accl_pitch_zero += Ad_pitch_Accl; delay(1); }
         Accl_pitch_zero = Accl_pitch_zero / 10;       // fetch zero
         // --------- test pitch ----------------------------
         Te = 2;
         while (Te > 0) {         // test pitch
            Ad_pitch_Accl = analogRead(Ad_channel_pitch_Accl);  Ad_pitch_Accl -= Accl_pitch_zero;
            if (Ad_pitch_Accl > 3) { Ad_pitch_Accl -= 3; Ad_pitch_Accl *= 10; Drive_a = Ad_pitch_Accl + 10; Drive_b = Drive_a * -1; }
            else { if (Ad_pitch_Accl < -3) { Ad_pitch_Accl += 3; Ad_pitch_Accl *= 10; Drive_a = Ad_pitch_Accl - 10; Drive_b = Drive_a * -1; } else {Drive_a=0;Drive_b=0;} }
         Set_pwm(); delay(1);F_switch();delay(10);}
         Serial.println ("Test Finished");Signal(); Signal();delay(3000);
         Drive_a = 0; Drive_b = 0; Set_pwm(); digitalWrite(LED6,HIGH);//reset leds to start condition
         digitalWrite(LED5,LOW);digitalWrite(LED4,LOW);digitalWrite(LED3,LOW);digitalWrite(LED2,LOW);digitalWrite(LED1,LOW);
            TCNT2 = 0;
            TCCR2A |= (1<<WGM21);                        // start timer 2
            TCCR2B |= (1<<CS20) | (1<<CS21) | (1<<CS22); // start timer 2
            TIMSK2 |= (1<<OCIE2A);                       // enable compare interrupt  
         Parser();      
         Process();
  }
}


ISR(TIMER2_COMPA_vect) {
/*-----------------------------------------------------------------------------
Function           : TIMER2_COMPA()
Timer loop every 10mSec
Date               : 25.09.2010
-----------------------------------------------------------------------------*/
    TIMSK2 &= ~(1<<OCIE2A); // disable interrupt
    Ad_swi = 1024 - analogRead(Ad_channel_foot);
    Get_pitch_angle();
    #if X_enable
      Get_roll_angle();
    #endif
    Algo();
    #if I2C_enable
      Gear();
    #endif
    Steering();
    Process();
    Set_pwm();
    Checkbatt();
    TIMSK2 |= (1<<OCIE2A);// enable interrupt
};
/*-----------------------------------------------------------------------------
Function           : Get_Tilt_pitch_angle()
calculate the Tilt Angle with a hand made Filter loop,
calculate the Angle Rate
Date               : 25.09.2010
-----------------------------------------------------------------------------*/
void Get_pitch_angle()
{
    Ad_pitch_gyro = analogRead(Ad_channel_pitch_gyro);
    #if !Zzaag
      Ad_pitch_gyro = 1024 - Ad_pitch_gyro;
    #endif
    #if !Inverse_poti
      Ad_steering = analogRead(Ad_channel_steering);
    #else {
       Ad_steering = 1024 - analogRead(Ad_channel_steering);
    #endif
    Ad_pitch_Accl = analogRead(Ad_channel_pitch_Accl);
    Ad_batt = analogRead(Ad_channel_batt);
//    Ad_swi = 1024 - analogRead(Ad_channel_foot);
    Ad_pitch_Accl = Ad_pitch_Accl * 10;
    Ad_pitch_gyro = Ad_pitch_gyro * 10;
    //subtract 1/150th of the value
    Buf = Total_pitch_Accl_gyro / Total_pitch_looptime;
    Total_pitch_Accl_gyro = Total_pitch_Accl_gyro - Buf;
    Buf = Ad_pitch_Accl - Accl_pitch_zero;
    
/* Leave speedlimit unused for now 
    #if I2C_enable
      // The software speedlimit (km/h * 2 * 5 * 10)
      if (No_speed_limit_flag = 0) {
         if (Speed_sum > 600) {Temp_Accl_pitch_zero++;}
         else {
            if (Temp_Accl_pitch_zero > 0) {
               Temp_Accl_pitch_zero = Temp_Accl_pitch_zero - 3;
               if (Temp_Accl_pitch_zero < 0) { Temp_Accl_pitch_zero = 0;}
            }
         }
         if (Temp_Accl_pitch_zero > 70) { Temp_Accl_pitch_zero = 70;}
         Buf = Buf + Temp_Accl_pitch_zero;
      }
    #endif
*/    
    Total_pitch_Accl_gyro = Total_pitch_Accl_gyro + Buf;
    // Filter for gyro
    Buf1 = Average_pitch_gyro / Total_pitch_looptime;
    Average_pitch_gyro = Average_pitch_gyro - Buf1;
    Average_pitch_gyro = Average_pitch_gyro + Ad_pitch_gyro;
    Buf1 = Average_pitch_gyro / Total_pitch_looptime10;
    // calculate the Angle Rate
    Buf = Ad_pitch_gyro * 10; 
    Buf1 = Buf1 - Buf;	
    Buf1 = Buf1 * 30;					
    Angle_pitch_rate = Buf1 / 100;
    // calculate the Tilt Angle
    // correct the Tilt_pitch_angle with the average of the gyro
    // The Accl-lowpass filter terminates all fast peaks
    // The Anglerate part rebuilds the correct flanks in Tilt_pitch_angle
    Total_pitch_Accl_gyro = Total_pitch_Accl_gyro + Angle_pitch_rate;
    Buf = Total_pitch_looptime10 * 10;
    Tilt_pitch_angle= Total_pitch_Accl_gyro / Buf;
    Angle_pitch_rate = Angle_pitch_rate / 10;
//    Serial.print("Tilt_pitch_angle = ");Serial.print(Tilt_pitch_angle);Serial.print("\tAngle_pitch_rate = ");Serial.println(Angle_pitch_rate);
}
/*-----------------------------------------------------------------------------
Function           : Get_rocker_angle()
calculate the Tilt Angle for the rocker,
same way as Get_Tilt_pitch_angle
Date               : 25.09.2010
-----------------------------------------------------------------------------*/
#if X_enable
void Get_roll_angle()
{   
//***********************************************************************
     Ad_roll_Accl = analogRead(Ad_channel_roll_Accl);
     Ad_roll_Accl = Ad_roll_Accl * 10 ;
     // subtract 1/20th of the value
     Buf_roll = Total_roll_Accl_gyro / Total_roll_looptime;
     Total_roll_Accl_gyro = Total_roll_Accl_gyro - Buf_roll;
     Buf_roll = Ad_roll_Accl - Accl_roll_zero;
     Total_roll_Accl_gyro = Total_roll_Accl_gyro + Buf_roll;

     Buf = Total_roll_looptime * 10;
     Buf = Buf / 2.7;
     Tilt_roll_angle = Total_roll_Accl_gyro / Buf;
     if (Tilt_roll_angle > 35) { Tilt_roll_angle = 35;}
     if (Tilt_roll_angle < -35) { Tilt_roll_angle = -35;}
     Tilt_roll_angle = Tilt_roll_angle / 2;
}
#endif
/*-----------------------------------------------------------------------------
Function           : Algo()
Calculation of motor current out of sensor values
Date               : 25.09.2010
-----------------------------------------------------------------------------*/
void Algo()
{
    Balance_moment =  Tilt_pitch_angle ;//+ Anglecorrection;  // Anglecorrection has no function
    Balance_moment =  Angle_pitch_rate + Balance_moment;      // calculate balance moment
    Balance_moment =  Balance_moment * 17;
    Drive_sum = Drive_sum + Balance_moment;
    if (Drive_sum >  55000) {Drive_sum =  55000;}  // Limit Drive_sum
    if (Drive_sum < -55000) {Drive_sum = -55000;}  // Limit Drive_sum
    Buf = Drive_sum / 155;
    Buf1 = Balance_moment / 165;
    Drivespeed = Buf + Buf1;                         // calculate Drive speed
}
/*-----------------------------------------------------------------------------
Function           : Steering()
Calculation of steering values
Date               : 25.09.2010
-----------------------------------------------------------------------------*/
void Steering ()
{
  Steeringsq = Steering_zero - Ad_steering;                                    // deviation from the steering zeropoint
  if ((Steeringsq >150) || (Steeringsq < -150)) { Steeringsq = Steeringold; }    // safety in case something goes wrong
  else { Steeringold = Steeringsq; }

  #if I2C_enable                                                         // Steering depends on speed (new and cool)
  Buf1 = Speed_sum /70;                                                  // Buf1 is speed in km/h * 0.71 (50/70) now (another fine adjustment screw?)
  Buf1 = abs (Buf1);
  if (Buf1 > 20) {Buf1=20;}                                              // 20/0.71 = 14.2 km/h dont decrease the steering sensitivity over 14.2 km/h
  Steering_sensitivity = 22 - Buf1;                                        // invert the value
  Steering_sensitivity = Steering_sensitivity /2;                            // the value at 0 km/h is 11, when speed is 14.2 or higher the value = 1
  if (Steeringsq >  45) {Steeringsq =  45;}
  if (Steeringsq < -45) {Steeringsq = -45;}
  Steering_position = Steeringsq;
  Steeringsq = Steeringsq * Steering_sensitivity;                       // insert the speeed-sensitive steering factor
  Steeringsq = Steeringsq / storage.Parameter1;                                      // Parameter1 is a screw for fineadjustment of the speed-sensitive steering
  Speed_error = Steeringsq - Speed_correction;                             // compares the speed difference with the handlebar position
  if (Speed_error >  30) { Speed_error =  30;}
  if (Speed_error < -30) { Speed_error = -30;}
  Steeringsq = Steeringsq + Speed_error;                                     // add the errorvalue
  #else 
  Buf1 = Drive_sum / 20000;                                              // Steering depends on power (old and ugly)
  Buf1 = abs (Buf1);
  if (Buf1 < 1 ) { Buf1 = 1; }
  Steeringsq = Steeringsq / Buf1;
  // some safety lines, limits steering: Drive_sum 55000 max = +-5
  Buf1 = Drive_sum / 2000;
  Buf1 = abs(Buf1);
  if (Buf1>22) { Buf1 = 22;}
  Buf1 = 27 - Buf1;
  Buf2 = Buf1 * -1;
  if (Steeringsq > Buf1) { Steeringsq = Buf1;}
  if (Steeringsq < Buf2) { Steeringsq = Buf2;} 
  #endif
  Steeringsignal = Steeringsq;
  Drive_a = Drivespeed - Steeringsignal;
  Drive_b = Drivespeed + Steeringsignal;
  #if Zzaag
    Drive_a = Drive_a * -1; // invert if ZZaag
    Drive_b = Drive_b * -1;
  #endif
}
/*-----------------------------------------------------------------------------
Function           : Process()
Processing Wheelie states
On standby delete all values, so the motors standing still
Date               : 25.09.2010
-----------------------------------------------------------------------------*/
void Process()
{
    switch (MMode){
    case Standby:                                           // Standby MMode    
        Drive_a = 0;
        Drive_b = 0;
        Drivespeed=0;  
        Drive_sum = 0;
        Tilt_pitch_angle = 0; 
        Total_pitch_Accl_gyro = 0;
        Statustext = "Standby";
        if(Ad_swi > Sw_down)                                     // If foot switch is pressed: change Mmode to run
        {           
            Init();  
            TCNT2 = 0;
            TCCR2A |= (1<<WGM21);                        // start timer 2
            TCCR2B |= (1<<CS20) | (1<<CS21) | (1<<CS22); // start timer 2
            TIMSK2 |= (1<<OCIE2A);                       // enable compare interrupt         
            MMode = Run;
            Statustext = "Drive!";
        }
    break;
    case Run:                                                    // Run operation
            if(Ad_swi < Sw_down)                                 // If foot switch is released: change Mmode to down
        {           
            MMode = Down;
        }
    break;    
    case Down:    
          Statustext = "Down !!!";                              //Added 23/4/2011
          Serial.print ("2:11=");Serial.print(Statustext);Serial.print("\r\n");delay(8);
        for (Buf1=1;Buf1<256;Buf1++){
          if ((Drive_a == 0) && (Drive_b == 0)) {continue;}
          if (Drive_a >0) { --Drive_a;}
          if (Drive_a <0) { ++Drive_a;}
          if (Drive_b >0) { --Drive_b;}
          if (Drive_b <0) { ++Drive_b;}
          delay(150);
          Set_pwm();
        }
//        TCCR2A &= ~(1<<WGM21);                        // stop timer 2
//        TCCR2B &= ~(1<<CS20) | (1<<CS21) | (1<<CS22); // stop timer 2
//        TIMSK2 &= ~(1<<OCIE2A);                       // disable compare interrupt
        MMode = Standby;
        Statustext = "Standby";
    break;
    default:
        MMode = Down;
    break;  
    }
}
/*-----------------------------------------------------------------------------
Function           : Set_pwm()
limiting PWM and Set the direction flags
Date               : 25.09.2010
-----------------------------------------------------------------------------*/
void Set_pwm()
{
    if(Drive_a >  Max_pwm) {Drive_a =  Max_pwm;}   // limiting PWM Signal A
    else {
    if(Drive_a < -Max_pwm) {Drive_a = -Max_pwm;}   // limiting PWM Signal A
    }
    if(Drive_b >  Max_pwm) {Drive_b =  Max_pwm;}   // limiting PWM Signal B
    else {
    if(Drive_b < -Max_pwm) {Drive_b = -Max_pwm;}   // limiting PWM Signal B
    }
    if(Drive_a < 0)                                // Check direction
    { digitalWrite(Cw_ccw_a,HIGH);}                // CW
    else
    { digitalWrite(Cw_ccw_a,LOW);}                 // CCW
    if(Drive_b < 0)                                // Check direction
    { digitalWrite(Cw_ccw_b,HIGH);}                // CW
    else
    { digitalWrite(Cw_ccw_b,LOW);}                 // CCW
    Drivea = abs (Drive_a);
    Driveb = abs (Drive_b);
    OCR1A = (255 - Drivea);                               // inverse Signal to have a Phase shift from 180 to Signal PWM B
    OCR1B = Driveb;                               // Set PWM

}
/*-----------------------------------------------------------------------------
Function           : Checkbatt()
calculate the Battery Voltage and display using the leds
Date               : 25.09.2010
-----------------------------------------------------------------------------*/
void Checkbatt()
{
    // moving average of battery voltage
    Buf1 = Average_batt / 300;
    Average_batt = Average_batt - Buf1;
    Average_batt = Average_batt + Ad_batt;
    Buf1 = Average_batt / 300;
    if (Buf1 > Battok)     { digitalWrite (LED1,HIGH);digitalWrite(LED2,HIGH);digitalWrite(LED3,HIGH);digitalWrite(LED4,HIGH);}
    if (Buf1 > Battupper)  { digitalWrite (LED1,LOW);digitalWrite(LED2,HIGH);digitalWrite(LED3,HIGH);digitalWrite(LED4,HIGH);}
    if (Buf1 > Battlower)  { digitalWrite (LED1,LOW);digitalWrite(LED2,LOW);digitalWrite(LED3,HIGH);digitalWrite(LED4,HIGH);}
    if (Buf1 < Battdead)   { digitalWrite (LED1,LOW);digitalWrite(LED2,LOW);digitalWrite(LED3,LOW);digitalWrite(LED4,HIGH);}
}
/*----------------------------------------------------------------------------
Function           : Init()
   Calibration: Calculation of steering mid and  platform horizontal position.
Date               : 25.09.2010
 ---------------------------------------------------------------------------*/
void Init()
{
   //Calculate mid position of steering (Steering_zero)
   Ad_steering = 0;
   for (Buf=1;Buf<11;Buf++) {                                        //Mean out of 10 values
      #if !Inverse_poti
      Ad_steering = Ad_steering + analogRead(Ad_channel_steering);
      #else
      Ad_steering = Ad_steering + 1024 - analogRead(Ad_channel_steering);
      #endif
      delay(1);
   }
   Steering_zero = Ad_steering / 10;
   //Calculate gyro horizontal position of platform
   Ad_pitch_gyro = 0;
   for (Buf=1;Buf < (Total_pitch_looptime + 1);Buf++) {
      Buf1 = analogRead(Ad_channel_pitch_gyro);
      #if !Zzaag
      Buf1 = 1024 - Buf1;
      #endif
      Buf1 = Buf1 * 10;
      Ad_pitch_gyro = Ad_pitch_gyro + Buf1;
      delay(2);
   }
   Gyro_pitch_zero = Ad_pitch_gyro / Total_pitch_looptime;
   Average_pitch_gyro = Ad_pitch_gyro;                                   //Preset Average_pitch_gyro

   //Calculate Accl horizontal position of platform
   Ad_pitch_Accl = 0;
   for (Buf=1;Buf<101;Buf++) {                                       //Mean out of 10 values
      Buf1 = analogRead(Ad_channel_pitch_Accl);
      Ad_pitch_Accl = Ad_pitch_Accl + Buf1;
//******************************************************************************      
//      Total_pitch_Accl_gyro = Ad_pitch_Accl;    //BK 22/4/2010 faster find state equilibrium
      delay(2);
   }
   Accl_pitch_zero = Ad_pitch_Accl / 10;
   #if X_enable
     // calculate gyro horizontal position of platform
     Ad_roll_gyro = 0;
     for (Buf=1;Buf < (Total_roll_looptime + 1);Buf++) {
//**********************************************************************************
       Buf1 = analogRead (Ad_channel_roll_gyro);
       Buf1 = Buf1 * 10;
       Ad_roll_gyro = Ad_roll_gyro + Buf1;
       delay(2);
     }
     Average_roll_gyro = Ad_roll_gyro;
     //calculate Accl horizontal position of platform
     Buf1=0;
     for (Buf = 1; Buf <101;Buf++) {
       Ad_roll_Accl = analogRead (Ad_channel_roll_Accl);
       Buf1 = Ad_roll_Accl + Buf1;
       delay(2);
     }
     Accl_roll_zero = Buf1 / 10;
   #endif
   //Check if values are in tolerable range
   if (Accl_pitch_zero < 3000) { Errno = 3; Serial.print ("2:11=Error with Accl"); Err_value(); }
   if (Accl_pitch_zero > 7000) { Errno = 3; Serial.print ("2:11=Error with Accl"); Err_value(); }
   if (Gyro_pitch_zero < 3000) { Errno = 2; Serial.print ("2:11=Error with Gyro"); Err_value(); }
   if (Gyro_pitch_zero > 7000) { Errno = 2; Serial.print ("2:11=Error with Gyro"); Err_value(); }
} 
/*----------------------------------------------------------------------------
Function           : Err_value()
  On error the program is processing this routine endlessly.
  Switch the Wheelie off to reset.
Date               : 07.08.2010
---------------------------------------------------------------------------*/
void Err_value()
{
  do {
    for (Buf=1;Buf<(Errno+1);Buf++) {
      digitalWrite(LED5,HIGH);
      delay(300);
      digitalWrite(LED5,LOW);
      delay(300);
    }
    delay(2000);
  }
    while(1);
}



/*---------------------------------------------------------------------------
     Gear
     Get values from the Geartoothsensors
---------------------------------------------------------------------------*/
#if I2C_enable
void Gear ()
{ 
   I2c.requestFrom(32, 9);    // request 9 bytes from slave device 32
   int i=0;
   while (I2c.available() > 0){
     Speed_array[i] = I2c.receive();
     i++;
   }
  // We get 9 bytes from sensor assuming we have a ATS657 sensor with direction
  // If we have ATS667 then this will not be present. so we have to ignore: !!!!!not done yet
  Speed_left = Wheel_const / Speed_left;                    // speed in km/h * 10
  Speed_right = Wheel_const / Speed_right;                  // speed in km/h * 10  
   if (Speed_left <250) {
    if (digitalRead(Cw_ccw_b) == 1) { Speed_left_out = Speed_left * -1; }
    else { Speed_left_out = Speed_left; }
  }
  if (Speed_right <250) {
    if (digitalRead(Cw_ccw_a) == 1) { Speed_right_out = Speed_right * -1; }
    else { Speed_right_out = Speed_right; }
  }     
  Buf = Speed_sum / 5;                                      // mean out of 5
  Speed_sum = Speed_sum - Buf;
  Buf = Speed_right_out + Speed_left_out;
  Speed_sum = Speed_sum + Buf;                              // 5 * 10 = 50 times of the average speed of the wheels
  
  Buf = Speed_diff / 5;                                     // mean out of 5
  Speed_diff = Speed_diff - Buf;
  Buf = Speed_left_out - Speed_right_out;
  Speed_diff = Speed_diff + Buf;                             // 5 * 10 = 50 times of the average speed of the wheels
  Speed_correction = Speed_diff / storage.Parameter2;
}
#endif
//---------------------------------------------------------------------------
//     Parser
//
//     The Parser cuts the string in Moduleadress,
//     Subchannel and Value.
//---------------------------------------------------------------------------
void Parser()
{ 
   // We are looking for command in form of 2:17=23
   // 17 is Channel and 23 is Val
   // If channel = 23 then data is stored in EEPROM. If channel is 24 then RESET to original values
   StringCounter=0;
   NextChar=-1; 
   if (Serial.available ()) { NextChar = Serial.read(); } else { NextChar = -1; }
   while (NextChar !=10 && NextChar !=-1 && StringCounter < 40)
   {
   if (NextChar != '\0'  && NextChar !=13 ) { Inputstring += char(NextChar);  }
   if (Serial.available ()) { NextChar = Serial.read(); delay(20);} else { NextChar =-1; }
   }
   if (NextChar == 10)  {
    //check if
    String left = Inputstring.substring(1,2);
    if (left == ":") { NextChar=58; }
    left = Inputstring.substring(2); // chop off 1st two characters. these are 2:
    int Channel = left.indexOf("=");   // find position of = sign
    int Value;
    String right=left.substring(Channel+1); // To the right of = is value
    left = left.substring(0,Channel);       // To the left of = sign is channel
    char channel[left.length()+1];        //convert characters to their values
    left.toCharArray(channel, sizeof(channel));
    Channel = atoi(channel);
    char value[right.length()+1];
    right.toCharArray(value, sizeof(value));
    Value = atoi(value);
    if (NextChar = 58 && StringCounter <= 39) {
       if (Channel == 17) storage.Parameter1 = Value;
       if (Channel == 18) storage.Parameter2 = Value;
       if (Channel == 19) storage.Parameter3 = Value;
       if (Channel == 20) storage.Parameter4 = Value;
       if (Channel == 21) storage.Parameter5 = Value;
       if (Channel == 22) storage.Parameter6 = Value;
       // some protection against silly values
       if (storage.Parameter1 > Parameter1_max){ storage.Parameter1 =  Parameter1_max;} 
       if (storage.Parameter1 < Parameter1_min){ storage.Parameter1 =  Parameter1_min;}
       if (storage.Parameter2 > Parameter2_max){ storage.Parameter2 =  Parameter2_max;}
       if (storage.Parameter2 < Parameter2_min){ storage.Parameter2 =  Parameter2_min;}
       if (storage.Parameter3 > Parameter3_max){ storage.Parameter3 =  Parameter3_max;}
       if (storage.Parameter3 < Parameter3_max){ storage.Parameter3 =  Parameter3_min;}       
       if (Channel == 23){         
       for (unsigned int t=0; t<sizeof(storage); t++) {
       EEPROM.write(CONFIG_START + t, *((char*)&storage + t));
       }  
            Serial.print ("2:11=");Serial.print ("STORED!\r\n");       
            delay(1); 
        }
       if (Channel == 24) {
            storage.Parameter1 = Parameter1;
            storage.Parameter2 = Parameter2;
            storage.Parameter3 = Parameter3;
            storage.Parameter4 = Parameter4;
            storage.Parameter5 = Parameter5;
            storage.Parameter6 = Parameter6 ;
            for (unsigned int t=0; t<sizeof(storage); t++) {
            EEPROM.write(CONFIG_START + t, *((char*)&storage + t));
            } 
            Serial.print ("2:11=");Serial.print ("RESET!\r\n");       
            delay(1);
       }
       if (Channel == 25) {
            testmode = !testmode; 
            if (testmode == false) {Serial.print ("2:11=");Serial.print ("TEST MODE OFF!\r\n");} 
            else {Serial.print ("2:11=");Serial.print ("TEST MODE ON!\r\n");}
            delay(1);
       }
    }
Inputstring[0] = '\0';
StringCounter = 0;
}
}

  void F_switch()
  {
    Ad_swi = 0;
    for (Buf=1;Buf<11;Buf++) {
      Ad_swi = Ad_swi + 1024 - analogRead(Ad_channel_foot);
      delay(1);
    }
    Ad_swi = Ad_swi/10;
    if (Ad_swi > 300) {
      Te = 0;
      while (Ad_swi > 50) {
        Ad_swi = 0;
        for (Buf=1;Buf<11;Buf++) {
         Ad_swi = Ad_swi + 1024 - analogRead(Ad_channel_foot);
         delay(1);
         }
        Ad_swi = Ad_swi/10;
        delay(300);// get rid bounce
      }
    }
    return;
  }

void Signal()
{
  #if Zzaag
    digitalWrite(Buzzer,HIGH);
    delay(100);
    digitalWrite(Buzzer,LOW);
    delay(100);
  #else
    digitalWrite(LED1,HIGH);
    delay(300);
    digitalWrite(LED1,LOW);
    delay(300);
    digitalWrite(LED1,HIGH);
    delay(300);
    digitalWrite(LED1,LOW); 
    delay(300);   
  #endif
}



