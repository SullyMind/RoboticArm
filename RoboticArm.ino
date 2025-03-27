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

// Orientation/Motion Variables
Quaternion q;
VectorFloat gravity;
float yrp[3];      // [yaw, roll, pitch]
float yaw, pitch;

typedef struct
{
  // MPU6050 Control/Status Variables
  bool DMPReady;  
  uint8_t devStatus;
  uint16_t packetSize;
  uint8_t FIFOBuffer[64]; 
  float yaw, pitch;
} imu_data;

/* -------------- IMU HAND ----------------- */

#define IMU_INT_PIN       2     // external interrupt pin

MPU6050 imu_hand; 
imu_data imu_hand_data;

// Interrupt detection routine
volatile bool imu_hand_interrupt = false;
void DMPDataReadyHand() {
  imu_hand_interrupt = true;
}

/* -------------- IMU ARM ----------------- */

#define IMU_ARM_INT_PIN   4     // external interrupt pin

MPU6050 imu_arm(0x69);
imu_data imu_arm_data;

// Interrupt detection routine
volatile bool imu_arm_interrupt = false;
void DMPDataReadyArm() {
  imu_arm_interrupt = true;
}

/*   -------------------------------------------
     |              Setup function             |
     -------------------------------------------
*/

void imu_init(MPU6050* imu, imu_data* data, int interrupt_pin, void (*interrupt_fct)(void))
{
  imu->initialize(); 
  pinMode(interrupt_pin, INPUT);

  Serial.println(imu->testConnection()?"MPU6050 connection successful":"MPU6050 connection failed"); 

  data->devStatus = imu->dmpInitialize();
  Serial.println(data->devStatus == 0?"DMP connection successful":"DMP connection failed"); 
  
  if (data->devStatus == 0) {
    imu->CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    imu->CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    imu->PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    imu->setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.println("Enabling external interrupt detection");
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), interrupt_fct, RISING);
    //MPUIntStatus = imu->getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println("DMP ready! Waiting for first interrupt...");
    data->DMPReady = true;
    data->packetSize = imu->dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(data->devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }

  if (!data->DMPReady) {
    Serial.println("IMU DMP is not ready. Something is wrong!");
  }
}

void setup() {
  Serial.begin(115200);

  /* -------------------------------------------
     |  Inertial Measurement Unit - MPU6050    |
     -------------------------------------------
  */
  Wire.begin(); // join I2C bus 
  Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  Wire.setWireTimeout(3000, true); // Needed to avoid freezes of program due to I2C

  imu_init(&imu_hand, &imu_hand_data, IMU_INT_PIN, DMPDataReadyHand);
  imu_init(&imu_arm, &imu_arm_data, IMU_ARM_INT_PIN, DMPDataReadyArm);

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
  if (imu_hand.dmpGetCurrentFIFOPacket(imu_hand_data.FIFOBuffer)) { // Get the Latest packet 
      imu_hand.dmpGetQuaternion(&q, imu_hand_data.FIFOBuffer);
      imu_hand.dmpGetGravity(&gravity, &q);
      imu_hand.dmpGetYawPitchRoll(yrp, &q, &gravity);
      yaw = yrp[0] * 180/M_PI;
      //roll = yrp[1] * 180/M_PI;
      pitch = yrp[2] * 180/M_PI;
      /*Serial.print("ypr\t");
      Serial.print(yaw);
      Serial.print("°\t");
      Serial.println(pitch);*/
      /*Serial.print("°\t");
      Serial.println(roll);*/

      if(abs(imu_hand_data.yaw-yaw) > 5)
      {
        imu_hand_data.yaw = yaw;
        long val = constrain(yaw, -90, 45);
        val = map(val, -90, 45, 0, 180);
        servo_rotation.write(val);
      }
      if(abs(imu_hand_data.pitch-pitch) > 5)
      {
        imu_hand_data.pitch = pitch;
        long val = constrain(pitch, -45, 45);
        val = map(val, -45, 45, 0, 180);
        servo_vertical_move.write(val);
        servo_horizontal_move.write(val);
      }
  }
  if (imu_arm.dmpGetCurrentFIFOPacket(imu_arm_data.FIFOBuffer)) {
    imu_arm.dmpGetQuaternion(&q, imu_arm_data.FIFOBuffer);
    imu_arm.dmpGetGravity(&gravity, &q);
    imu_arm.dmpGetYawPitchRoll(yrp, &q, &gravity);
    yaw = yrp[0] * 180/M_PI;
    pitch = yrp[2] * 180/M_PI;

    Serial.print("ypr\t");
    Serial.print(yaw);
    Serial.print("°\t");
    Serial.println(pitch);
  }
  delay(100);
}