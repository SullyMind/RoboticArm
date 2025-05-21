#ifndef COMMON_H
#define COMMON_H

typedef enum {
  GRIP,
  HORIZONTAL,
  VERTICAL,
  ROTATION
} id_t; 

byte angle;

byte rx_addr[] = "robot"; 

#endif