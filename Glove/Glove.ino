#include <Servo.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include "mrf24j.h"
#include "common.h"

/* -------------------------------------------
   |         Flex Sensor variables           |  
   -------------------------------------------
*/

#define FLEX_SENSOR_PIN   A1

/* -------------------------------------------
   |      IMU - MPU6050 variables            |  
   -------------------------------------------
*/

// Orientation/Motion Variables
Quaternion q;
VectorFloat gravity;
float ypr[3];      // [yaw, pitch, roll]
float yaw, pitch, roll;

typedef struct
{
  // MPU6050 Control/Status Variables
  bool DMPReady;  
  uint8_t devStatus;
  uint16_t packetSize;
  uint8_t FIFOBuffer[64]; 
  float yaw, pitch, roll;
} imu_data;

/* -------------- IMU HAND ----------------- */

#define IMU_INT_PIN       5     // external interrupt pin

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

/* -------------------------------------------
   |          MRF24J RF Module               |  
   -------------------------------------------
*/

const int pin_reset = 6;
const int pin_cs = 10; // default CS pin on ATmega8/168/328
const int pin_interrupt = 2; // default interrupt pin on ATmega8/168/328

Mrf24j mrf(pin_reset, pin_cs, pin_interrupt);

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
     |           MRF24J RF Module              |
     -------------------------------------------
  */
  mrf.reset();
  mrf.init();
  mrf.set_pan(0xcafe);
  mrf.address16_write(0x6002); // address
  mrf.set_promiscuous(true);
  attachInterrupt(0, interrupt_routine, CHANGE); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328

  interrupts(); // Enable interrupts
}

/* -------------------------------------------
   |       MRF24J RF Module Functions        |
   -------------------------------------------
  */

void interrupt_routine() {
    mrf.interrupt_handler(); // mrf24 object interrupt routine
}

void rx_handler() {
    Serial.println("MRF24J RX Handler");
    Serial.print("DATA: "); 
    for (int i = 0; i < mrf.rx_datalength(); i++) {
        Serial.write(mrf.get_rxinfo()->rx_data[i]);
    }
    Serial.println("");
}

void tx_handler() {
    if (mrf.get_txinfo()->tx_ok) {
        Serial.println("TX went ok, got ack");
    } else {
        Serial.print("TX failed after ");
        Serial.print(mrf.get_txinfo()->retries);
        Serial.println(" retries\n");
    }
}

void rf_send(id_t rf_id, byte val)
{
  char rf_data[2] = {rf_id, val};
  mrf.send16(0x6002, rf_data);
}
 
/*   -------------------------------------------
     |              Loop function              |
     -------------------------------------------
*/
void loop() {
  //scan for rx and tx
  mrf.check_flags(&rx_handler, &tx_handler); 

  // Flex sensor read
  int p = analogRead(FLEX_SENSOR_PIN); 
  angle = map(p, 380, 550, 0, 180);
  angle = constrain(angle, 0, 180); // make sure angle is kept in valid range
  rf_send(GRIP, angle);

  // Read a packet from IMU DMP FIFO 
  if (imu_hand.dmpGetCurrentFIFOPacket(imu_hand_data.FIFOBuffer)) { // Get the Latest packet 
      imu_hand.dmpGetQuaternion(&q, imu_hand_data.FIFOBuffer);
      imu_hand.dmpGetGravity(&gravity, &q);
      imu_hand.dmpGetYawPitchRoll(ypr, &q, &gravity);
      yaw = ypr[0] * 180/M_PI;
      pitch = ypr[1] * 180/M_PI;
      roll = ypr[2] * 180/M_PI;
      
      /*Serial.print("ypr\t");
      Serial.print(yaw);
      Serial.print("°\t");
      Serial.print(pitch);
      Serial.print("°\t");
      Serial.println(roll);*/

      if(abs(imu_hand_data.yaw-yaw) > 5)
      {
        imu_hand_data.yaw = yaw;
        long val = constrain(yaw, -90, 45);
        angle = map(val, -90, 45, 0, 180);
        rf_send(ROTATION, angle);
      }
      if(abs(imu_hand_data.roll-roll) > 5)
      {
        imu_hand_data.roll = roll;
        long val = constrain(roll, -45, 45);
        // Adaptive ranges depending on another servomotor  
        if (imu_arm_data.roll < 30)
          angle = map(val, -45, 45, 20, 100);
        else if (imu_arm_data.roll > 65)
          angle = map(val, -45, 45, 10, 70);
        else
          angle = map(val, -45, 45, 0, 120);
        rf_send(HORIZONTAL, angle);
      }
  }
  if (imu_arm.dmpGetCurrentFIFOPacket(imu_arm_data.FIFOBuffer)) {
    imu_arm.dmpGetQuaternion(&q, imu_arm_data.FIFOBuffer);
    imu_arm.dmpGetGravity(&gravity, &q);
    imu_arm.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = ypr[0] * 180/M_PI;
    pitch = ypr[1] * 180/M_PI;
    roll = ypr[2] * 180/M_PI;

    Serial.print("ypr\t");
    Serial.print(yaw);
    Serial.print("°\t");
    Serial.print(pitch);
    Serial.print("°\t");
    Serial.println(roll);

    if(abs(imu_arm_data.roll-roll) > 5)
    {
      imu_arm_data.roll = roll;
      long val = constrain(roll, -90, 0);
      angle = map(val, 0, -90, 10, 80);
      rf_send(VERTICAL, angle);
    }
  }
  delay(10);
}