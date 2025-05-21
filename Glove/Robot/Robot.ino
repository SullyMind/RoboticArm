#include <Servo.h>
#include <SPI.h>
#include "mrf24j.h"
#include "common.h"

/* -------------------------------------------
   |          MRF24J RF Module               |  
   -------------------------------------------
*/

const int pin_reset = 6;
const int pin_cs = 10; // default CS pin on ATmega8/168/328
const int pin_interrupt = 2; // default interrupt pin on ATmega8/168/328

Mrf24j mrf(pin_reset, pin_cs, pin_interrupt);

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

void setup() {
  Serial.begin(115200);

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

  /* -------------------------------------------
     |         Servo Motors / Setup            |
     -------------------------------------------
  */
  servo_vertical_move.attach(3); 
  servo_horizontal_move.attach(5); 
  servo_rotation.attach(6);
  servo_grip.attach(9);
}

/* -------------------------------------------
   |       MRF24J RF Module Functions        |
   -------------------------------------------
*/

void interrupt_routine() {
  mrf.interrupt_handler(); // mrf24 object interrupt routine
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

char rx_buf[2];

void rx_handler() {
  Serial.println("MRF24J RX Handler");
  Serial.print("DATA: ");
  for (int i = 0; i < mrf.rx_datalength(); i++) {
    Serial.write(mrf.get_rxinfo()->rx_data[i]);
    Serial.print(" ");
  }
  Serial.println("");

  // Sanity check
  if (mrf.rx_datalength() == 2){
    id_t id = mrf.get_rxinfo()->rx_data[0];
    angle = mrf.get_rxinfo()->rx_data[1];

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
  } else {
    Serial.println("RX data length is not matching expected one");
  }
}

/*   -------------------------------------------
     |              Loop function              |
     -------------------------------------------
*/
void loop() {
  mrf.check_flags(&rx_handler, &tx_handler);
  delay(10);
}