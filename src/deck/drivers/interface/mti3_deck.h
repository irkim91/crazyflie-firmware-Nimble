#ifndef MTI3_DECK_H
#define MTI3_DECK_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    // Quaternion
    float q0, q1, q2, q3;
    float roll, pitch, yaw;

    // Rate of Turn (Gyro)
    float gyr[3]; 

    // Acceleration
    float acc[3]; 

    // [추가됨] Magnetic Field (a.u. or Gauss)
    float mag[3]; 

    uint16_t packetCounter;
    bool isUpdated; 
} Mti3Data_t;

bool mti3GetData(Mti3Data_t* outData);
bool mti3IsConnected(void);

#endif // MTI3_DECK_H