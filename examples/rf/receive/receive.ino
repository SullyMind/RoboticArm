/**
 * Example code for using a microchip mrf24j40 module to send and receive
 * packets using plain 802.15.4
 * Requirements: 3 pins for spi, 3 pins for reset, chip select and interrupt
 * notifications
 * This example file is considered to be in the public domain
 * Originally written by Karl Palsson, karlp@tweak.net.au, March 2011
 */
#include <SPI.h>  // SPI library
#include "mrf24j.h" // RF module library.
#include <Servo.h>

Servo servo1;
int angle = 0;  // Angle initial du servo
char txt;

const int pin_reset = 6;
const int pin_cs = 10; // default CS pin on ATmega8/168/328
const int pin_interrupt = 2; // default interrupt pin on ATmega8/168/328



Mrf24j mrf(pin_reset, pin_cs, pin_interrupt);


long last_time;
long tx_interval = 1000;

void setup() {
  servo1.attach(5);          // PWM 5


  Serial.begin(9600); 
  
  mrf.reset();
  mrf.init();
  
  mrf.set_pan(0xcafe); // network ID
  // This is _our_ address
  mrf.address16_write(0x6002); // node address

  // uncomment if you want to receive any packet on this channel
  mrf.set_promiscuous(true);
  
  // uncomment if you want to enable PA/LNA external control
  //mrf.set_palna(true);

 
  // uncomment if you want to buffer all PHY Payload
  //mrf.set_bufferPHY(true);

  attachInterrupt(0, interrupt_routine, CHANGE); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
  last_time = millis();
  interrupts();
}

void interrupt_routine() {
    mrf.interrupt_handler(); // mrf24 object interrupt routine
}

void loop() {

    mrf.check_flags(&rx_handler, &tx_handler);
    unsigned long current_time = millis();
    if (current_time - last_time > tx_interval) {
        last_time = current_time;
        //Serial.println("\r\broadcasting/awaiting message...");

        
    }



}

void rx_handler() {
    Serial.print("\nreceived a packet ");Serial.print(mrf.get_rxinfo()->frame_length, DEC);Serial.println(" bytes long");
    
    if(mrf.get_bufferPHY()){
      Serial.println("Packet data (PHY Payload):");
      for (int i = 0; i < mrf.get_rxinfo()->frame_length; i++) {
          Serial.print(mrf.get_rxbuf()[i]);
      }
    }
    
    Serial.println("\r\nASCII data (relevant data):");
    String data = "";

// Construire la String à partir des données reçues
for (int i = 0; i < mrf.rx_datalength(); i++) {
    char c = mrf.get_rxinfo()->rx_data[i];
    if (isPrintable(c)) {  // Ignore les caractères non imprimables
        data += c;
    }
}

Serial.print("Donnée reçue : ");
Serial.println(data);

// Nettoyage : supprimer les caractères parasites
data.trim(); // Supprime espaces, \n, \r au début/fin

// Conversion
int valeur = data.toInt();  // Convertit la string en int

Serial.print("Valeur convertie : ");
Serial.println(valeur);

// Contrôle du servo
servo1.write(valeur);
delay(2000);
    /*
    Serial.print("\r\nLQI/RSSI=");
    Serial.print(mrf.get_rxinfo()->lqi, DEC);
    Serial.print("/");
    Serial.println(mrf.get_rxinfo()->rssi, DEC);*/
}

void tx_handler() {
    if (mrf.get_txinfo()->tx_ok) {
        Serial.println("TX went ok, got ack");
    } else {
        Serial.print("TX failed after ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" retries\n");
    }
}


