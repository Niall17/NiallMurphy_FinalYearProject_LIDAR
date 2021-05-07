#include <TFMPlus.h>  // Include TFMini Plus Library v1.4.1
TFMPlus tfmP;         // Create a TFMini Plus object
TFMPlus tfmP2;
TFMPlus tfmP3;

#include "printf.h"   // Modified to support Intel based Arduino
                      // devices such as the Galileo. Download from:
                      // https://github.com/spaniakos/AES/blob/master/printf.h

// The Software Serial library is an alternative for devices that
// have only one hardware serial port. Delete the comment slashes
// on lines 37 and 38 to invoke the library, and be sure to choose
// the correct RX and TX pins: pins 10 and 11 in this example. Then
// in the 'setup' section, change the name of the hardware 'Serial2'
// port to match the name of your software serial port, such as:
// 'mySerial.begin(115200); etc.

#include <SoftwareSerial.h>       
SoftwareSerial mySerialC( 12,13);   
SoftwareSerial mySerialB( 10,11);
SoftwareSerial mySerialA( 8,9); 

// Define Constants
 
// Connections to A4988
const int dirPin = 2;  // Direction
const int stepPin = 3; // Step
const int enable = 4; // enable


// Motor steps per rotation
const unsigned int STEPS_PER_REV = 50; //50 steps at 1.8deg = 90deg

float anglef = 0; //integer for angle
int anglei=0;
boolean debug=false;
int debugCount=0;

//this line of code will keep the previous x value and not reinitialise on reset of Arduino, thus allowing for reallignment of steppers 
__attribute__((section(".noinit"))) unsigned int x;
                                    
void setup()
{
    
  
     // Setup the pins as Outputs
    pinMode(stepPin,OUTPUT); 
    pinMode(dirPin,OUTPUT);
    pinMode(enable,OUTPUT);
    digitalWrite(enable,HIGH); //immediately disable the stepper motors from being able to move (enable pin is active low)
    digitalWrite(stepPin,LOW); //immediately write step pin low to try prevent any unwanted sporatic movement in the beginning

    Serial.begin( 115200);   // Intialize terminal serial port
    delay(20);               // Give port time to initalize
    printf_begin();          // Initialize printf.
    
    //reset protocol at beginning of setup to home steppers if required
    if ((x >= STEPS_PER_REV)){ 
        x = 0;    //first check if step angle is greater than number of steps
        
    }
    else if(x!=0){
        Serial.println("else");
        digitalWrite(enable,LOW);         //enable steppers momentarily for reset 
        digitalWrite(dirPin,HIGH);        // Set motor direction clockwise to return to home position

        digitalWrite(stepPin,HIGH);             //rotate steppers clockwise until home (0 step) reached
        delayMicroseconds(2000);
        digitalWrite(stepPin,LOW); 
        delayMicroseconds(2000);
        
        while(x!=0){
            digitalWrite(stepPin,HIGH);             //rotate steppers clockwise until home (0 step) reached
            delayMicroseconds(2000);
            digitalWrite(stepPin,LOW); 
            delayMicroseconds(2000);
            x=x-1;                                  //decrement step until zero reached
            delay(50);
            Serial.println(x);
            
        }
        digitalWrite(enable,HIGH);        //disable steppers for setup
    } 
    
    
  
    

    //wait for pi to begin process
    while(1){

      if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n');
        
        if(data.equals("y")){
          debug=true;
          Serial.println("Test Mode Selected");
          break;
        }
        else if (data.equals("n")){
          debug=false;
          debugCount=5;
          Serial.println("Normal Mode Selected");
          break;
        }
        
      }
      
    }

    
    printf("\r\nTFMPlus Library Example - 18JUN2020\r\n");  // say 'hello'

    mySerialA.begin( 115200);  // Initialize TFMPLus device serial ports.
    delay(20);                // Give port time to initalize
    tfmP.begin( &mySerialA);   // Initialize device library object and...
                                // pass device serial port to the object.

    // Send some example commands to the TFMini-Plus
    // - - Perform a system reset - - - - - - - - - - -
    printf( "System reset: ");
    if( tfmP.sendCommand( SYSTEM_RESET, 0))
    {
        printf( "passed.\r\n");
    }
    else tfmP.printReply();
  
    delay(500);  // added to allow the System Rest enough time to complete

  // - - Display the firmware version - - - - - - - - -
    printf( "Firmware version: ");
    if( tfmP.sendCommand( OBTAIN_FIRMWARE_VERSION, 0))
    {
        printf( "%1u.", tfmP.version[ 0]); // print three single numbers
        printf( "%1u.", tfmP.version[ 1]); // each separated by a dot
        printf( "%1u\r\n", tfmP.version[ 2]);
    }
    else tfmP.printReply();
    // - - Set the data frame-rate to 250Hz - - - - - - - - to match 4000us delay at steppers
    printf( "Data-Frame rate: ");
    if( tfmP.sendCommand( SET_FRAME_RATE, FRAME_250))
    {
        printf( "%2uHz.\r\n", FRAME_250);
    }
    else tfmP.printReply();
    // - - - - - - - - - - - - - - - - - - - - - - - -
    
    mySerialB.begin( 115200); //repeat process for lidar B
    delay(20);  
    tfmP2.begin( &mySerialB);  

    delay(500);            // And wait for half a second.

    // Send some example commands to the TFMini-Plus
    // - - Perform a system reset - - - - - - - - - - -
    printf( "System reset: ");
    if( tfmP2.sendCommand( SYSTEM_RESET, 0))
    {
        printf( "passed.\r\n");
    }
    else tfmP2.printReply();
  
    delay(500);  // added to allow the System Rest enough time to complete

  // - - Display the firmware version - - - - - - - - -
    printf( "Firmware version: ");
    if( tfmP2.sendCommand( OBTAIN_FIRMWARE_VERSION, 0))
    {
        printf( "%1u.", tfmP2.version[ 0]); // print three single numbers
        printf( "%1u.", tfmP2.version[ 1]); // each separated by a dot
        printf( "%1u\r\n", tfmP2.version[ 2]);
    }
    else tfmP2.printReply();
    // - - Set the data frame-rate to 20Hz - - - - - - - -to match 4000us delay at steppers
    printf( "Data-Frame rate: ");
    if( tfmP2.sendCommand( SET_FRAME_RATE, FRAME_250))
    {
        printf( "%2uHz.\r\n", FRAME_250);
    }
    else tfmP2.printReply();
    // - - - - - - - - - - - - - - - - - - - - - - - -

    delay(500);            // And wait for half a second.
    
    mySerialC.begin( 115200); //repeat intialization process for Lidar C
    delay(20);  
    tfmP3.begin( &mySerialC);                        

    // Send some example commands to the TFMini-Plus
    // - - Perform a system reset - - - - - - - - - - -
    printf( "System reset: ");
    if( tfmP3.sendCommand( SYSTEM_RESET, 0))
    {
        printf( "passed.\r\n");
    }
    else tfmP3.printReply();
  
    delay(500);  // added to allow the System Rest enough time to complete

  // - - Display the firmware version - - - - - - - - -
    printf( "Firmware version: ");
    if( tfmP3.sendCommand( OBTAIN_FIRMWARE_VERSION, 0))
    {
        printf( "%1u.", tfmP3.version[ 0]); // print three single numbers
        printf( "%1u.", tfmP3.version[ 1]); // each separated by a dot
        printf( "%1u\r\n", tfmP3.version[ 2]);
    }
    else tfmP3.printReply();
    // - - Set the data frame-rate to 20Hz - - - - - - - -to match 4000us delay at steppers
    printf( "Data-Frame rate: ");
    if( tfmP3.sendCommand( SET_FRAME_RATE, FRAME_250))
    {
        printf( "%2uHz.\r\n", FRAME_250);
    }
    else tfmP3.printReply();
    // - - - - - - - - - - - - - - - - - - - - - - - -

    delay(500);            // And wait for half a second.

    printf("start\r\n"); //signal pi to begin reading the data from loop

    digitalWrite(enable,LOW); //enable steppers once setup complete
}

// Initialize variables
int16_t tfDistA = 0;    // Distance to object in centimeters
int16_t tfFluxA = 0;    // Strength or quality of return signal
int16_t tfTempA = 0;    // Internal temperature of Lidar sensor chip
int16_t tfDistB = 0;    
int16_t tfFluxB = 0;   
int16_t tfTempB = 0;   
int16_t tfDistC = 0;    
int16_t tfFluxC = 0;    
int16_t tfTempC = 0;   

// Use the 'getData' function to pass back device data.
void loop()
{
  delay(50);      //delay for one moment between loops

  if(debugCount==10){  //used to prevent x from counting upwards forever when not in debug mode
    debugCount=5;
  }
    
  if((debug=true) && (debugCount==3)){   //if in debug mode, 3 scans at a time
        Serial.println("wait");
        while(1){
          
          if (Serial.available() > 0) {
              String data = Serial.readStringUntil('\n');
              if(data.equals("y")){   //on user prompt, scan again if in debug mode
                debugCount=0;         //reset debug count
                break;
              }
      } 
    }   
  }
  // Set motor direction COUNTER-clockwise
  digitalWrite(dirPin,LOW); 
    
// scan room counter clockwise
  for(x = 0; x < STEPS_PER_REV; x++) {
    //delay(50);   
                                 
    digitalWrite(stepPin,HIGH);             //stepper delays sum to 4000us to match 250Hz frame rate of Lidar's
    delayMicroseconds(2000);
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(2000);

    anglef = x*1.8;
    anglei = round(anglef);
    
    mySerialA.listen();
    while(!mySerialA.available() ){}             //do nothing until serial data available
    tfmP.getData( tfDistA, tfFluxA, tfTempA); // Get data from the device A.

    mySerialB.listen();
    while(!mySerialB.available() ){}             //do nothing until serial data available
    tfmP2.getData( tfDistB, tfFluxB, tfTempB); // Get data from the device B.

    mySerialC.listen();
    while(!mySerialC.available() ){}             //do nothing until serial data available
    tfmP3.getData( tfDistC, tfFluxC, tfTempC); // Get data from the device C.

    
    printf( "DistA: %04i cm DistB: %04i cm DistC: %04i cm Angle: %03i deg\r\n", tfDistA,tfDistB,tfDistC,anglei);   // SEND SERIALLY TO PI
    
  }
  
  // Pause for one MOMENT
  delay(50); 
  
  // Set motor direction clockwise
  digitalWrite(dirPin,HIGH);
  
  // Scan room clockwise
  for(x = STEPS_PER_REV; x > 0; x--) {
    //delay(50);
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(2000);

    anglef = x*1.8;
    anglei = round(anglef);
    
    mySerialA.listen();
    while(!mySerialA.available() ){}             //do nothing until serial data available
    tfmP.getData( tfDistA, tfFluxA, tfTempA); // Get data from the device A.

    mySerialB.listen();
    while(!mySerialB.available() ){}             //do nothing until serial data available
    tfmP2.getData( tfDistB, tfFluxB, tfTempB); // Get data from the device B.

    mySerialC.listen();
    while(!mySerialC.available() ){}             //do nothing until serial data available
    tfmP3.getData( tfDistC, tfFluxC, tfTempC); // Get data from the device C.

    
    printf( "DistA: %04i cm DistB: %04i cm DistC: %04i cm Angle: %03i deg\r\n", tfDistA,tfDistB,tfDistC,anglei);   // SEND SERIALLY TO PI
    
  }

  debugCount++;
    
    
}
