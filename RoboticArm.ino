/* Install from Library Manager MPU6050 library by Electronic Cats */

#include <Servo.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* -------------------------------------------
   |      Servo motors variables             |  
   -------------------------------------------
*/

Servo servo_vertical_move;
Servo servo_horizontal_move;
Servo servo_rotation;
Servo servo_grip;

/* -------------------------------------------
   |      IMU - MPU6050 variables            |  
   -------------------------------------------
*/

#define IMU_INT_PIN 2 // external interrupt pin

MPU6050 imu; 

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64]; 

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

/*---Orientation/Motion Variables---*/ 
Quaternion q;
VectorFloat gravity;
float ypr[3];           // [yaw, pitch, roll]
float yaw, pitch;
float current_yaw, current_pitch;

/*   -------------------------------------------
     |              Setup function             |
     -------------------------------------------
*/

void setup() {
  Serial.begin(115200);

  /* -------------------------------------------
     |  Inertial Measurement Unit - MPU6050    |
     -------------------------------------------
  */
  Wire.begin(); // join I2C bus 
  Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  Wire.setWireTimeout(3000, true); // Needed to avoid freezes of program due to I2C

  imu.initialize(); 
  pinMode(IMU_INT_PIN, INPUT);

  Serial.println(imu.testConnection()?"MPU6050 connection successful":"MPU6050 connection failed"); 

  devStatus = imu.dmpInitialize();
  Serial.println(devStatus == 0?"MPU6050 connection successful":"MPU6050 connection failed"); 
  
  if (devStatus == 0) {
    imu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    imu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    imu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    imu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.println("Enabling external interrupt detection");
    attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), DMPDataReady, RISING);
    MPUIntStatus = imu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println("DMP ready! Waiting for first interrupt...");
    DMPReady = true;
    packetSize = imu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }

  if (!DMPReady) {
    Serial.println("IMU DMP is not ready. Something is wrong!");
  }

  /* -------------------------------------------
     |         Servo Motors / Setup            |
     -------------------------------------------
  */
  servo_vertical_move.attach(3); 
  servo_horizontal_move.attach(5); 
  servo_rotation.attach(6);
  servo_grip.attach(9);
}
 
/*   -------------------------------------------
     |              Loop function              |
     -------------------------------------------
*/
void loop() {
  /* Read a packet from IMU DMP FIFO */
  if (imu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
      imu.dmpGetQuaternion(&q, FIFOBuffer);
      imu.dmpGetGravity(&gravity, &q);
      imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      yaw = yrp[0] * 180/M_PI;
      roll = yrp[1] * 180/M_PI;
      pitch = yrp[2] * 180/M_PI;
      Serial.print("ypr\t");
      Serial.print(yaw);
      Serial.print("°\t");
      Serial.print(pitch);
      Serial.print("°\t");
      Serial.println(roll);

      if(abs(yaw-current_yaw) > 5)
      {
        current_yaw = yaw;
        long val = constrain(yaw, -90, 45);
        val = map(val, -90, 45, 0, 180);
        Serial.print("Mapped yaw: ");
        Serial.println(val);
        servo_rotation.write(val);
      }
      if(abs(pitch-current_pitch) > 5)
      {
        current_pitch = pitch;
        long val = constrain(pitch, -45, 45);
        val = map(val, -45, 45, 0, 180);
        Serial.print("Mapped pitch: ");
        Serial.println(val);
        servo_vertical_move.write(val);
        servo_horizontal_move.write(val);
      }
  }
  delay(100);
}