#ifndef _IN_GYRO_NSWITCH_H
#define _IN_GYRO_NSWITCH_H

// Nintendo Switch only: reads the gyro straight from libnx (HID six-axis sensor),
// because SDL's Switch backend does not report one.
// Deliberately uses no game types so that it can include <switch.h> without clashes.

// starts the six-axis sensors of the handheld unit, a Pro Controller and Joy-Cons for player 1
// returns 1 if the sensors could be set up, 0 if not
int nxGyroInit(void);

// reads the first connected sensor (handheld, Pro Controller, then right and left Joy-Con)
// values are in raw libnx angular velocity units (1.0 = one full rotation per second, to be verified):
//   pitch: rotation around the device's X axis (positive looks up)
//   yaw:   rotation around the direction of gravity, so it stays a left/right turn however the device is held
//   roll:  rotation around the device's Z axis (only for diagnostics)
// returns 1 if a connected sensor was read, 0 if not
int nxGyroRead(float *pitch, float *yaw, float *roll);

#endif
