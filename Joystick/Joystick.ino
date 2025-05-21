#include "nRF24L01.h"
#include "RF24.h"
#include "SPI.h"
#include "Robot/common.h"

#define DEBUG

#define CE_PIN  9
#define CSN_PIN 10

#define BUTTONA_PIN         2 // Button Blue - A
#define BUTTONB_PIN         3 // Button Yellow - B
#define BUTTONC_PIN         4 // Button Blue - C 
#define BUTTOND_PIN         5 // Button Yellow - D 
#define BUTTONE_PIN         7 // SMD button E on pcb
#define BUTTONF_PIN         6 // SMD button F on pcb
#define BUTTONJOYSTICK_PIN  8 // Button in joystick
#define X_PIN               A0
#define Y_PIN               A1

int x,y;
int btn_a, btn_c;
int servo_rotation_angle, servo_vertical_angle;
int servo_horizontal_angle, servo_horizontal_angle_prev;
int val;

int buttons[]={ BUTTONA_PIN, BUTTONB_PIN, BUTTONC_PIN, BUTTOND_PIN, BUTTONE_PIN, BUTTONF_PIN, BUTTONJOYSTICK_PIN };

RF24 radio(CE_PIN,CSN_PIN);

void setup(){
  for (int i=0; i <7 ; i++) {
    pinMode(buttons[i], INPUT_PULLUP);
    digitalWrite(buttons[i], HIGH);  
  }
  servo_horizontal_angle_prev = servo_horizontal_angle = 45;

#ifdef DEBUG
  Serial.begin(115200);
#endif

  // Setup nRF24...
  radio.begin();
  radio.setDataRate(RF24_2MBPS);
  radio.setRetries(3,5); // delay, count
  radio.openWritingPipe(rx_addr);
}

void rf_send(id_t rf_id, byte val)
{
  char data[2] = {rf_id, val};
  radio.write(data, 2);
}

void loop(){
  // Read digital buttons...
  btn_a = digitalRead(BUTTONA_PIN);
  if (btn_a == 0)
  {
    servo_horizontal_angle++;
  }
  btn_c = digitalRead(BUTTONC_PIN);
  if (btn_c == 0)
  {
    servo_horizontal_angle--;
  }
  if (abs(servo_horizontal_angle_prev-servo_horizontal_angle) > 5)
  {
    servo_horizontal_angle_prev = servo_horizontal_angle;
    // Due to robot mechanical constraints, horizontal angle depends on vertical angle
    if (servo_vertical_angle < 30)
    {
      servo_horizontal_angle = constrain(servo_horizontal_angle, 20, 100);
    }
    else if (servo_vertical_angle > 65)
    {
      servo_horizontal_angle = constrain(servo_horizontal_angle, 10, 70);
    }
    else
    {
      servo_horizontal_angle = constrain(servo_horizontal_angle, 0, 120);
    }
    // Send value to robotic arm
    rf_send(HORIZONTAL, servo_horizontal_angle);
#ifdef DEBUG
    Serial.print("Horizontal angle: ");
    Serial.print(servo_horizontal_angle);
    Serial.println();
#endif
  }

  // Read joystick values...
  val = analogRead(X_PIN);
  if (abs(x-val) > 5)
  {
    x = val;
    // Convert to servo angle
    val = constrain(x, 0, 663);
    servo_rotation_angle = map(val, 0, 663, 0, 180);
    // Send value to robotic arm
    rf_send(ROTATION, servo_rotation_angle);
#ifdef DEBUG
    Serial.print("Rotation angle: ");
    Serial.print(servo_rotation_angle);
    Serial.println();
#endif
  }
  val = analogRead(Y_PIN);
  if (abs(y-val) > 5)
  {
    y = val;
    // Convert to servo angle
    val = constrain(y, 0, 663);
    servo_vertical_angle = map(val, 0, 663, 10, 80);
    // Send value to robotic arm
    rf_send(VERTICAL, servo_vertical_angle);
#ifdef DEBUG
    Serial.print("Vertical angle: ");
    Serial.print(servo_vertical_angle);
    Serial.println();
#endif
  }

  delay(10);
}
