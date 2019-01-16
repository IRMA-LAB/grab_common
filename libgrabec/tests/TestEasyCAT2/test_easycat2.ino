//*********************************************************************************************

#define CUSTOM                      // Custom mode
#include "TestEasyCAT2.h"           // This file has been created by the Easy Configurator 
                                    // and must be located in the Arduino project folder
                                    //
                                    // There are two others files created by the Easy Configurator:
                                    // TestEasyCAT2.bin that must be loaded into the EEPROM.
                                    // TestEasyCAT2.xml that must be used by the EtherCAT master. 
                                    
//*********************************************************************************************

#include "EasyCAT.h"                // EasyCAT library to interface the LAN9252
#include <SPI.h>                    // SPI library

 
EasyCAT EASYCAT;                    // EasyCAT istantiation

                                    // The constructor allow us to choose the pin used for the EasyCAT SPI chip select 
                                    // Without any parameter pin 9 will be used 
                   
                                    // We can choose between:
                                    // 8, 9, 10, A5, 6, 7                                    

                                    // On the EasyCAT board the SPI chip select is selected through a bank of jumpers                Millis = millis();                                    // For this application we choose a cycle time of 150 mS

                                    // (The EasyCAT board REV_A allows only pins 8, 9, 10 through 0 ohm resistors)

 //EasyCAT EASYCAT(8);              // example:                                  
                                    // pin 8 will be used as SPI chip select
                                    // The chip select chosen by the firmware must match the setting on the board  


//---- pins declaration ------------------------------------------------------------------------------
int LedRed = 2;     // the PWM pin the LED is attached to
int LedOrange = 3;    // the PWM pin the LED is attached to
int LedYellow = 4;      // the PWM pin the LED is attached to


//---- global variables ---------------------------------------------------------------------------
uint8_t BrightnessRed = 0;        // how bright the LED is
uint8_t BrightnessOrange = 0;     // how bright the LED is
uint8_t BrightnessYellow = 0;       // how bright the LED is
uint64_t counter = 0;


// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);                                             // serial line initialization
                                                                  //(used only for debug)
           
  Serial.print ("\nEasyCAT - Generic EtherCAT slave\n");          // print the banner
  
  // declare pins to be an outputs:
  pinMode(LedRed, OUTPUT);
  pinMode(LedOrange, OUTPUT);
  pinMode(LedYellow, OUTPUT);                                                                    
                                                                  //---- initialize the EasyCAT board -----
                                                                  
  if (EASYCAT.Init() == true)                                     // initialization
  {                                                               // succesfully completed
    Serial.println("initialized");                                //
  }                                                               //
  
  else                                                            // initialization failed   
  {                                                               // the EasyCAT board was not recognized
    Serial.println("initialization failed");                      //     
                                                                  // The most common reason is that the SPI 
                                                                  // chip select choosen on the board doesn't 
                                                                  // match the one choosen by the firmware
                                                                  
                                                                  // stay in loop for ever
                                                                  // with the red led blinking
    while(1)                                                      //
    {                                                             //   
      digitalWrite (LedRed, LOW);                                 // 
      delay(500);                                                 //   
      digitalWrite (LedRed, HIGH);                                //  
      delay(500);                                                 // 
    }                                                             // 
  }  
}

//---- main loop ---------------------------------------------------------------------------------------- 
void loop()                                             // In the main loop we must call ciclically the 
{                                                       // EasyCAT task and our application
                                                        //
                                                        // This allows the bidirectional exachange of the data
                                                        // between the EtherCAT master and our application
                                                        //
                                                        // The EasyCAT cycle and the Master cycle are asynchronous
                                                        //   

  EASYCAT.MainTask();                                   // execute the EasyCAT task
  
  application();                                        // user applications

}

// the loop routine runs over and over again forever:
void application() {
  // read easycat input
  int16_t SensorValue = EASYCAT.BufferOut.Cust.Analog;
  //int16_t SensorValue = counter % 1024;  
  
  BrightnessRed = min(255, (int) (SensorValue / 341.0 * 255.0));
  BrightnessOrange = min(255, max(0, (int16_t) ((SensorValue - 341) / 341.0 * 255.0)));
  BrightnessYellow = min(255, max(0, (int16_t) ((SensorValue - 682) / 341.0 * 255.0)));
  
  EASYCAT.BufferIn.Cust.LedRed = BrightnessRed;
  EASYCAT.BufferIn.Cust.LedOrange = BrightnessOrange;
  EASYCAT.BufferIn.Cust.LedYellow = BrightnessYellow;
  
  if (counter % 200 == 0)
  {
    Serial.print(SensorValue);
    Serial.print("  ");
    Serial.print(BrightnessRed);
    Serial.print("  ");
    Serial.print(BrightnessOrange);
    Serial.print("  ");
    Serial.println(BrightnessYellow);
  }
  
  // set the brightness of pins:
  analogWrite(LedRed, BrightnessRed);
  analogWrite(LedOrange, BrightnessOrange);
  analogWrite(LedYellow, BrightnessYellow);

  counter ++;
  // for stability
  delay(1);
}
