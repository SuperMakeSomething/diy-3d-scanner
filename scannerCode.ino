/* Arduino Pro Micro Scanner Code (DIY 3D Scanner - Super Make Something Episode 8) - https://youtu.be/-qeD2__yK4c
 * by: Alex - Super Make Something
 * date: January 2nd, 2016
 * license: Creative Commons - Attribution - Non-Commercial.  More information available at: http://creativecommons.org/licenses/by-nc/3.0/
 */
 
 // Includes derivative of "ReadWrite" by David A. Mellis and Tom Igoe available at: https://www.arduino.cc/en/Tutorial/ReadWrite
 // Includes derivative of EasyDriver board sample code by Joel Bartlett available at: https://www.sparkfun.com/tutorials/400

/*
 * This code contains the follow functions:
 * - void setup(): initializes Serial port, SD card
 * - void loop(): main loop
 * - double readAnalogSensor(): calculates sensed distances in cm.  Sensed values calculated as an average of noSamples consecutive analogRead() calls to eliminate noise
 * - void writeToSD(): Writes sensed distance in cm to SD card file specified by filename variable
 * - void readFromSD(): Prints out contents of SD card file specified by filename variable to Serial
 */
 
 /* 
 * Pinout:
 * SD card attached to SPI bus as follows:
 * - MOSI - pin 16
 * - MISO - pin 14
 * - CLK - pin 15
 * - CS - pin 10
 * 
 * IR Sensor (SHARP GP2Y0A51SK0F: 2-15cm, 5V) attached to microcontroller as follows:
 * - Sense - A3
 * 
 * Turntable stepper motor driver board:
 * - STEP - pin 2
 * - DIR - pin 3 
 * - MS1 - pin 4
 * - MS2 - pin 5
 * - Enable - pin 6
 * 
 * Z-Axis stepper motor driver board:
 * - STEP - pin 7
 * - DIR - pin 8
 * - MS1 - pin 9
 * - MS2 - pin 18 (A0 on Arduino Pro Micro silkscreen)
 * - ENABLE - pin 19 (A1 on Arduino Pro Micro silkscreen)
 */

#include <SPI.h>
#include <SD.h>

File scannerValues;
String filename="scn000.txt";
int csPin=10;
int sensePin=A3;

int tStep=2;
int tDir=3;
int tMS1=4;
int tMS2=5;
int tEnable=6;

int zStep=7;
int zDir=8;
int zMS1=9;
int zMS2=18;
int zEnable=19;


void setup() 
{ 

  //Define stepper pins as digital output pins
  pinMode(tStep,OUTPUT);
  pinMode(tDir,OUTPUT);
  pinMode(tMS1,OUTPUT);
  pinMode(tMS2,OUTPUT);
  pinMode(tEnable,OUTPUT);
  pinMode(zStep,OUTPUT);
  pinMode(zDir,OUTPUT);
  pinMode(zMS1,OUTPUT);
  pinMode(zMS2,OUTPUT);
  pinMode(zEnable,OUTPUT);

  //Set microstepping mode for stepper driver boards.  Using 1.8 deg motor angle (200 steps/rev) NEMA 17 motors (12V)

/*
  // Theta motor: 1/2 step micro stepping (MS1 High, MS2 Low) = 0.9 deg/step (400 steps/rev)
  digitalWrite(tMS1,HIGH);
  digitalWrite(tMS2,LOW);
*/
  
  
  // Theta motor: no micro stepping (MS1 Low, MS2 Low) = 1.8 deg/step (200 steps/rev)
  digitalWrite(tMS1,LOW);
  digitalWrite(tMS2,LOW);
  
  
  // Z motor: no micro stepping (MS1 Low, MS2 Low) = 1.8 deg/step (200 steps/rev) --> (200 steps/1cm, i.e. 200 steps/10 mm).  Therefore 0.05mm linear motion/step.
  digitalWrite(zMS1,LOW);
  digitalWrite(zMS2,LOW);

  //Set rotation direction of motors
  //digitalWrite(tDir,HIGH);
  //digitalWrite(zDir,LOW);
  //delay(100);

  //Enable motor controllers
  digitalWrite(tEnable,LOW);
  digitalWrite(zEnable,LOW);
    
  // Open serial communications
  Serial.begin(9600);

  //Debug to Serial
  Serial.print("Initializing SD card... ");
  if (!SD.begin(csPin))
  {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization success!");
  
}

void loop() 
{

  int vertDistance=10; //Total desired z-axis travel
  int noZSteps=20; //No of z-steps per rotation.  Distance = noSteps*0.05mm/step
  int zCounts=(200/1*vertDistance)/noZSteps; //Total number of zCounts until z-axis returns home
  //int thetaCounts=400;
  int thetaCounts=200;

  // Scan object
  digitalWrite(zDir,LOW); 
  for (int j=0; j<zCounts; j++) //Rotate z axis loop
  {
    for (int i=0; i<thetaCounts; i++)   //Rotate theta motor for one revolution, read sensor and store
    {
      rotateMotor(tStep, 1); //Rotate theta motor one step
      delay(200);
      double senseDistance=0; //Reset senseDistanceVariable;
      senseDistance=readAnalogSensor(); //Read IR sensor, calculate distance
      writeToSD(senseDistance); //Write distance to SD
    }
  
    rotateMotor(zStep, noZSteps); //Move z carriage up one step
    delay(1000);
    writeToSD(9999); //Write dummy value to SD for later parsing
  }

  // Scan complete.  Rotate z-axis back to home and pause.
  digitalWrite(zDir,HIGH);
  delay(10);  
  for (int j=0; j<zCounts; j++)
  {
    rotateMotor(zStep, noZSteps);
    delay(10);
  }

  for (int k=0; k<3600; k++) //Pause for one hour (3600 seconds), i.e. freeze until power off because scan is complete.
  {
    delay(1000); 
  }

  //readFromSD(); //Debug - Read from SD 
}

void rotateMotor(int pinNo, int steps)
{
  
  for (int i=0; i<steps; i++)
  {
    digitalWrite(pinNo, LOW); //LOW to HIGH changes creates the
    delay(1);
    digitalWrite(pinNo, HIGH); //"Rising Edge" so that the EasyDriver knows when to step.
    delay(1);
    //delayMicroseconds(500); // Delay so that motor has time to move
    //delay(100); // Delay so that motor has time to move
  }
}

double readAnalogSensor()
{

  //int noSamples=10;  
  int noSamples=100;
  int sumOfSamples=0;

  int senseValue=0;
  double senseDistance=0;
  
  for (int i=0; i<noSamples; i++)
  {
    senseValue=analogRead(sensePin); //Perform analogRead
    delay(2); //Delay to let analogPin settle -- not sure if necessary
    sumOfSamples=sumOfSamples+senseValue; //Running sum of sensed distances
  }
  
  senseValue=sumOfSamples/noSamples; //Calculate mean
  senseDistance=senseValue; //Convert to double
  senseDistance=mapDouble(senseDistance,0.0,1023.0,0.0,5.0); //Convert analog pin reading to voltage
  //Serial.print("Voltage: ");     //Debug
  //Serial.println(senseDistance);   //Debug
  //Serial.print(" | "); //Debug
  senseDistance=-5.40274*pow(senseDistance,3)+28.4823*pow(senseDistance,2)-49.7115*senseDistance+31.3444; //Convert voltage to distance in cm via cubic fit of Sharp sensor datasheet calibration
  //Print to Serial - Debug
  //Serial.print("Distance: ");    //Debug
  //Serial.println(senseDistance); //Debug
  //Serial.print(senseValue);
  //Serial.print("   |   ");
  //Serial.println(senseDistance);
  return senseDistance;
}

void writeToSD(double senseDistance)
{
  // Open file
  scannerValues = SD.open(filename, FILE_WRITE);
  
  // If the file opened okay, write to it:
  if (scannerValues) 
  {
    //Debug to Serial
    /* 
     Serial.print("Writing to ");
     Serial.print(filename);
     Serial.println("..."); 
    */

    //Write to file
    scannerValues.print(senseDistance);
    scannerValues.println();
    
    // close the file:
    scannerValues.close();
    
    //Debug to Serial
    //Serial.println("done.");
  } 
  else 
  {
    // if the file didn't open, print an error:
    Serial.print("error opening ");
    Serial.println(filename);
  }
}

void readFromSD()
{  
  // Open the file for reading:
  scannerValues = SD.open(filename);
  if (scannerValues)
  {
    Serial.print("Contents of ");
    Serial.print(filename);
    Serial.println(":");

    // Read from file until there's nothing else in it:
    while (scannerValues.available()) 
    {
      Serial.write(scannerValues.read());
    }
    // Close the file:
    scannerValues.close();
  } 
  else
  {
    // If the file didn't open, print an error:
    Serial.print("error opening ");
    Serial.println(filename);
  }
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

