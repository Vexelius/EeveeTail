#include <Wire.h>
#include <VarSpeedServo.h>
#include <ADXL345.h>
#include <math.h>
#define SAMPLES 15


ADXL345 accel;      // Instance of the ADXL345 library
int xAxisMov = 80;  // Controls the position of the second segment of the tail
int yAxisMov = 90;  // Controls the position of the first segment of the tail

VarSpeedServo tailBone1;  // Servo connected to pin 11 - Controls the first segment of the tail (Y axis)
VarSpeedServo tailBone2;  // Servo connected to pin 10 - Controls the second segment of the tail (X axis)

// These variables store the acceleration in each one of the axes
// (Provided by the accelerometer module)
double debugAx[7];
double debugAy[7];
double debugAz[7];

// Variables are used to generate the average values
//Accumulators
double accX = 0;
double accY = 0;
double accZ = 0;
//Averages
double avgX;
double avgY;
double avgZ;

// Variables that hold the previous accelerometer values
double prevX = 0;
double prevY = 0;
double prevZ = 0;
// Variables that store the difference between the current and previous accelerometer values
double diffX = 0;
double diffY = 0;
double diffZ = 0;

// Switches to change the tail's direction
boolean tailBone1Ctrl = true;  //True: Up     | False: Down     ::      Up = 135  | Down = 45
boolean tailBone2Ctrl = true;  //True: Right  | False: Left     ::      Right = 0 | Left = 180

// This switch is used to start the comparison algorithm,
//because you need to generate at least one set of values in
//order to start comparing.
boolean heurXYZ = false;


double gatherAverage(double * average_x, double * average_y, double * average_z)
{
    Serial.println("Gathering Values...");
    double accum_x = 0, accum_y = 0, accum_z = 0; //Accumulators.
    //These variables will hold the values given by each axis
    //until the end of the sample size is reached

    //Data gathering loop
    for (uint8_t i = 0; i < SAMPLES; ++i)
    {
        int x,y,z;
        accel.readXYZ(&x, &y, &z);

        Serial.println("Raw values: ");
        Serial.print(x);
        Serial.print(" | " );
        Serial.print(y);
        Serial.print(" | ");
        Serial.println(z);

        accum_x += x;
        accum_y += y;
        accum_z += z;

        Serial.println("Accumulator values: ");
        Serial.print(accum_x);
        Serial.print(" | " );
        Serial.print(accum_y);
        Serial.print(" | ");
        Serial.println(accum_z);

        // sensor runs at 25hz by default
        // wait atleast 1/25th of a second before attempting
        // another read or we will read the same value
        delay(100);
    }

    //Obtaining the average for each axis
    *average_x = accum_x / SAMPLES;
    *average_y = accum_y / SAMPLES;
    *average_z = accum_z / SAMPLES;


    Serial.println("Averages Obtained:");
    Serial.print("X: ");
    Serial.print(*average_x);
    Serial.print(" Y: ");
    Serial.print(*average_y);
    Serial.print(" Z: ");
    Serial.println(*average_z);

    return 0;
}



void setup(){
  Serial.begin(9600);

  // + Accelerometer Setup
  accel.powerOn();  //Initialize the accelerometer
  accel.setRangeSetting(2); //Set the sensitivity to 2g
  accel.setAxisOffset(111,44,-128); //Set default range

  //set activity/ inactivity thresholds (0-255)
  accel.setActivityThreshold(75); //62.5mg per increment
  accel.setInactivityThreshold(75); //62.5mg per increment
  accel.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  //look for activity on the following axes - 1 == on; 0 == off 
  accel.setActivityX(1);
  accel.setActivityY(1);
  accel.setActivityZ(1);
 
  //look for inactivity on the following axes - 1 == on; 0 == off
  accel.setInactivityX(0);
  accel.setInactivityY(0);
  accel.setInactivityZ(0);
 
  //look of Taps on the following axes - 1 == on; 0 == off
  accel.setTapDetectionOnX(0);
  accel.setTapDetectionOnY(0);
  accel.setTapDetectionOnZ(0);
 
  //Set the sensitivity for Tap / Double Tap detection (0-255)
  accel.setTapThreshold(50); //62.5mg per increment
  accel.setTapDuration(15); //625us per increment
  accel.setDoubleTapLatency(80); //1.25ms per increment
  accel.setDoubleTapWindow(200); //1.25ms per increment
 
  //Set the sensitivity for Freefall (0-255)
  accel.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  accel.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  //Setting all interrupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  accel.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  accel.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  accel.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  accel.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  accel.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interrupt actions - 1 == on; 0 == off  
  accel.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  accel.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  accel.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  accel.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  accel.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);


  delay(2000); //This will give you enough time to get ready for the calibration routine
  calibrateOffsets();


  // Attach the servos for each bone segment
  delay(250);
  tailBone1.attach(11);
  tailBone2.attach(10);

  // TailBone1 speed: 15-25
  // TailBone2 speed: 50 

  tailBone1.write(90, 25, true);
  tailBone2.write(90, 50, true);
  delay(250);
}

void loop(){ 
  int i,j;
  //First, get samples
  for (i = 0; i < 7; i++)
  {
    double xyz[3];
    accel.getAcceleration(xyz);
    debugAx[i] = xyz[0];
    debugAy[i] = xyz[1];
    debugAz[i] = xyz[2];
    delay(25);
  }

  //Then, get an average value
  accX = 0;
  accY = 0;
  accZ = 0;

  for (i = 0; i < 7; i++)
  {
  accX = accX + debugAx[i];
  accY = accY + debugAy[i];
  accZ = accZ + debugAz[i];
  }

  avgX = accX/7;
  avgY = accY/7;
  avgZ = accZ/7;

  //When running for the first time, load the 
  //obtained values into prevX, prevY and PrevZ
  if(heurXYZ == false)
  {
    prevX = avgX;
    prevY = avgY;
    prevZ = avgZ;

    heurXYZ = true;
  }

  //Get the difference between previous and current values
  diffX = avgX - prevX;
  diffY = avgY - prevY;
  diffZ = avgZ - prevZ;

  //Print debug info
  Serial.print(" ( DiffX = ");
  Serial.print(diffX);
  Serial.print(" + DiffY = ");
  Serial.print(diffY);
  Serial.print(" + DiffZ = ");
  Serial.print(diffZ);
  Serial.println(" )");

   prevX = avgX;
   prevY = avgY;
   prevZ = avgZ;

  Serial.print(" [ X = ");
  Serial.print(avgX);
  Serial.print(" | Y = ");
  Serial.print(avgY);
  Serial.print(" | Z = ");
  Serial.print(avgZ);
  Serial.println(" ]");

  //And try to identify the movement
  if(heurXYZ==true)
  {
    if(
      ((diffX>=-0.1)&&(diffX<=0.11))
    &&((diffY>=-0.1)&&(diffY<=0.11))
    &&((diffZ>=-0.05)&&(diffZ<=0.05))
    )
    {
      Serial.println("< STATUS: STILL >");

      
      if((avgZ>=0.92)&&(avgZ<=1.10))
      {
        Serial.println("< STATUS: STANDING >");
        tailBone1.write(90, 45, false);
        if(tailBone2Ctrl == true)
        {
          xAxisMov = 0;
        }
        if(tailBone2Ctrl == false)
        {
          xAxisMov = 180;
        }
        tailBone2.write(xAxisMov, 50, true);
        tailBone2Ctrl = !tailBone2Ctrl;
      }
      if((avgZ>=0.55)&&(avgZ<=0.91))
      {
        Serial.println("< STATUS: SITTING >");
        tailBone1.write(135, 45, false);
        tailBone2.write(90, 50, true);
      }
    }
    
    else
    {
      tailBone1.write(135, 45, false);
      if(tailBone2Ctrl == true)
      {
      xAxisMov = 0;
      }
      if(tailBone2Ctrl == false)
      {
      xAxisMov = 180;
      }
      tailBone2.write(xAxisMov, 50, true);
      tailBone2Ctrl = !tailBone2Ctrl;
      Serial.println("< STATUS: MOVE >");
    }
  } 
}


void calibrateOffsets()
{
    Serial.println("Calibration in process. Please wait...");
    accel.setRangeSetting(2);
    accel.setAxisOffset(0,0,0);
    double average_x, average_y, average_z;

    if (gatherAverage(&average_x, &average_y, &average_z) != 0)
        return;

    Serial.println();
    Serial.println("Calculating Offsets...");

    int8_t offset_x, offset_y, offset_z;

    // we have to divide by four because the offsent sensitivity
    // is 15.6mg vs 3.9mg of the 2g sensitivity
    offset_x = 0 - round((double)average_x / 4);
    offset_y = 0 - round((double)(average_y - 256) / 4);
    offset_z = 0 - round((double)average_z / 4);

    Serial.println("Offset values:");
    Serial.print("X: ");
    Serial.print(offset_x);
    Serial.print(" Y: ");
    Serial.print(offset_y);
    Serial.print(" Z: ");
    Serial.println(offset_z);


    Serial.println();
    Serial.print("Setting Offsets with \"acc.setOffsets(");
    Serial.print(offset_x);
    Serial.print(", ");
    Serial.print(offset_y);
    Serial.print(", ");
    Serial.print(offset_z);
    Serial.print(");\"");
    Serial.println();
    Serial.println();

    accel.setAxisOffset(offset_x, offset_y, offset_z);
}
