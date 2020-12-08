#include <MPU6050.h> //include this library to use the gyroscope

#include <Wire.h> //this library allows you to communicate with I2C / TWI devices

#include <math.h> //use this library for quick maths

#include <AFMotor.h>



#define MPU6050_SAMPLERATE_DELAY_MS (1)//makes the gyroscope give an output every 100 ms

#define OUTPUTMAX 255

#define OUTPUTMIN 0

float KP = 5;

float KI = 4;

float KD = 1;

AF_DCMotor leftmotor (4);

AF_DCMotor rightmotor (1);

float thetaM; //measured angle of roll
float phiM; //measured angle of pitch

float thetaFold = 0; //filtered angle of roll

float thetaFnew;

float phiFold = 0;  //filtered angle of pitch

float phiFnew;

float thetaG = 0; //angle of roll measured by the gyro

float phiG = 0; //angle of pitch measured by the gyro

float dt;

float DT;

float theta = 0;

float phi = 0;

unsigned long millisOld;// a variable for the amount of time

unsigned long millisNew;

float pitchTarget = 0;

float pitchError;

float pitchErrorOld;

float pitchErrorChange;

float pitchErrorSlope;

float pitchErrorArea;

float motorSpeedBoth = 120; //speed with which the motor turns in PWM

float actualMotorSpeedBoth;

float output;
float setpoint =  0; //the desired angle in degrees
float input = theta;

MPU6050 mpu = MPU6050(); //declaring the gyroscope as mpu

void setup() {

  Serial.begin(115200); //sets the data rate of the serialport in baud

  mpu.begin();

  delay(1000); //wait 1000 ms


  Serial.println("mpu start");

  mpu.calibrateGyro();

  Serial.println("Gyro calbibrated");

  Serial.println("print;");

  delay(1000);

  Serial.println("Speed, theta, phi"); //print a description for the measurements

  Serial.println();



  millisOld = millis();



  leftmotor.run(FORWARD);

  rightmotor.run(FORWARD);

  leftmotor.setSpeed(motorSpeedBoth);

  rightmotor.setSpeed(motorSpeedBoth);
//drive forward with the designated speed
}







void loop() {

  millisOld = millisNew;

  millisNew = millis();

  dt = (millisNew - millisOld);

  


  Vector Acc = mpu.readNormalizeAccel(); //declare a vector Acc which is the normalized acceleration read by the accelerometer

  Vector Gyro = mpu.readNormalizeGyro(); //declare a vector Gyro which is the normalized data read by the gyroscope

  thetaM = -atan2(Acc.XAxis / 9.81 , Acc.ZAxis / 9.81) / 2 / 3.141592654 * 360; //a calculation to find a variable theta for the angle of tilt in degrees

  phiM = -atan2((Acc.YAxis - 14) / 9.81 , Acc.ZAxis / 9.81) / 2 / 3.141592654 * 360; // a calculation to find a variable phi for the angle of tilt in degrees

  phiFnew = .75 * phiFold + .25 * phiM; //"low pass filter" filters short peaks of outputs for a more stable result by giving more credit too the old value than to the new measurement

  thetaFnew = .75 * thetaFold + .25 * thetaM;

  DT = dt / 1000.; //convert dt from milliseconds to seconds

  thetaG = thetaG + Gyro.YAxis * DT; //using the Gyro measurements to create a angle called thetaGyro

  phiG = phiG - Gyro.XAxis * DT; //using the Gyro measurements to create a angle called phiGyro

  theta = (theta + Gyro.YAxis * DT) * .75 + thetaM * .25; //combining Gyro measurements and accelerometer measurements

  phi = (phi - Gyro.XAxis * DT) * .75 + phiM * .25; //P1 and P2 can be changed based on the delay

  Acc.YAxis != Acc.ZAxis != Acc.XAxis; //without this the y also gives output when the mpu moves over the other axes

  //this is a weird feature of our mpu which we can't explain (for now)



  phiFold = phiFnew; //update the old filtered value to the new value

  thetaFold = thetaFnew;



  pitchErrorOld = pitchError;

  pitchError = pitchTarget - phi;

  pitchErrorChange = pitchError - pitchErrorOld;

  pitchErrorSlope = pitchErrorChange / dt; //derivative

  pitchErrorArea = pitchErrorArea + pitchError * dt; //calculating the values used in de PID




  if (phi > 3) { //when the cart leans forward, drive forward

    actualMotorSpeedBoth = motorSpeedBoth + ( KP * pitchError + KI * pitchErrorSlope + KD * pitchErrorArea);

    leftmotor.run(FORWARD);

    rightmotor.run(FORWARD);

    leftmotor.setSpeed(actualMotorSpeedBoth);

    rightmotor.setSpeed(actualMotorSpeedBoth);

  }

  else if (phi < -3) { 

    actualMotorSpeedBoth = motorSpeedBoth - ( KP * pitchError + KI * pitchErrorSlope + KD * pitchErrorArea);

    leftmotor.run(BACKWARD);

    rightmotor.run(BACKWARD);

    leftmotor.setSpeed(actualMotorSpeedBoth);

    rightmotor.setSpeed(actualMotorSpeedBoth);

  }

  else {

    leftmotor.run(FORWARD);

    rightmotor.run(FORWARD);

    leftmotor.setSpeed(0);

    rightmotor.setSpeed(0);

  } 



  Serial.print(actualMotorSpeedBoth);

  Serial.print(",");

  Serial.print(theta);

  Serial.print(",");

  Serial.print(phi);

  Serial.print(",");

  Serial.print(KP);

  Serial.print(",");

  Serial.print(KI);

  Serial.print(",");

  Serial.println(KD);

//print some of the used variables

  if (Serial.available()) { //this part of the code is used for adjusting the values of KP, KI and KD using the command bar in the serial monitor

    String command = Serial.readStringUntil('\n');

    if (command == "KP +") {

      KP = KP + 0.01;

    }

    else if (command == "KP -") {

      KP = KP - 0.01;

    }

    else if (command == "KI -") {

      KI = KI - 1;

    }

    else if (command == "KI +") {

      KI = KI + 1;

    }

    if (command == "KD +") {

      KD = KD + 0.0001;

    }

    else if (command == "KD -") {

      KD = KD - 0.0001;

    }

  }

  

  //there is a limit at ca. 45-60 degrees on the angles that can be reached otherwise the measurements won't be accurate

  delay(MPU6050_SAMPLERATE_DELAY_MS); //delay of a certain lenght between each cycle of measurements

}
