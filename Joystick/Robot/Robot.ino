#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "common.h"

#define DEBUG

/* -------------------------------------------
   |         nRF24L01 RF Module              |  
   -------------------------------------------
*/

#define CE_PIN  9
#define CSN_PIN 10

char dataReceived[2];

RF24 radio(CE_PIN, CSN_PIN);

/* -------------------------------------------
   |      Servo motors variables             |  
   -------------------------------------------
*/

Servo servo_vertical_move;
Servo servo_horizontal_move;
Servo servo_rotation;
Servo servo_grip;

/*   -------------------------------------------
     |              Setup function             |
     -------------------------------------------
*/

void setup() 
{
#ifdef DEBUG
  Serial.begin(115200);
#endif

  Serial.println("Robotic Arm");

  /* -------------------------------------------
     |          nRF24L01 RF Module             |
     -------------------------------------------
  */
  radio.begin();
  radio.setDataRate(RF24_2MBPS);
  radio.openReadingPipe(1, rx_addr);
  radio.startListening();

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
  if ( radio.available() ) {
    Serial.println("RX");
    radio.read(&dataReceived, sizeof(dataReceived));

    id_t id = dataReceived[0];
    angle = dataReceived[1];

#ifdef DEBUG
        Serial.print("ID, angle: ");
        Serial.print(id);
        Serial.print(",");
        Serial.print(angle);
        Serial.println();
#endif

    switch(id)
    {
      case GRIP:
        servo_grip.write(angle);
        break;
      case HORIZONTAL:
        servo_horizontal_move.write(angle);
        break;
      case VERTICAL:
        servo_vertical_move.write(angle);
        break;
      case ROTATION:
        servo_rotation.write(angle);
        break;
    }
  }
  delay(1);
}