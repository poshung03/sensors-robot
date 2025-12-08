#ifndef __buzzertone__
#define __buzzertone__

#include <Arduino.h>
#include "arduinoFFT.h"

/*
 *  [0]--|||--[1]
 *   |         |
 *   |         |
 *  [0]       [1]
 *   |         |
 *   |         |
 *  [0]-------[1]
 */

/** Set the pins for the motors */
/** Set the positive and negative directions for the motors */

void writefreq(uint8_t freqwrite);
float ListenFreq(uint8_t numAverages = 4);

#endif // __CAR_CONTROL_H__

