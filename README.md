# AS5047 

AS5047 - a Hall effect on axis angular position sensor widely used in motion control systems - SPI interface library for Mbed-os 6 (baremetal and multithread)

## Authors

- [haarkon](https://github.com/haarkon)
- Henoc Mukumbi


# Documentation

File AS5047.h contain a fully written DOXYGEN documentation 

# Usage/Examples

```c++
/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "AS5047.h"
#include <chrono>

#define WAIT_TIME_MS 500

DigitalOut led1(LED1);
AS5047 sensor (SPI_MOSI, SPI_MISO, SPI_SCK, PF_14, 10000000);
Timer chronometer;

void errorHandling (int* errorCount);
long incrementalAngleHandling (short angle, AS5047::T_AS5047_Rotation direction);

int main()
{
    bool ready;
    int  errorCount = 0;
    AS5047::T_AS5047_Error cr;

    long angularPosition = 0;   // "long" is suitable for less than 131 072 turn else choose "long long". Avoid floating point numbers.
    short angle;
    AS5047::T_AS5047_Rotation direction;
    
    Watchdog &theDog = Watchdog::get_instance();
    theDog.start(10000);

    printf("This is the bare metal blinky example running on Mbed OS %d.%d.%d.\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);

    do {
        cr = sensor.isAS5047Ready(&ready);
    } while ((cr != 0) || (!ready));

    chronometer.start();
    chronometer.reset();

    while (true)
    {
      theDog.kick();
      while (chronometer.elapsed_time().count() < 1000); // 1ms wait between update
      chronometer.reset();

      if (errorCount > 0) errorCount--;

      cr = sensor.getAngularPosition(&angle); // get angle
      if (cr != AS5047_ERR_OK) errorHandling(&errorCount);

      cr = sensor.getDirection(&direction); // get direction
      if (cr != AS5047_ERR_OK) errorHandling(&errorCount);

      angularPosition += incrementalAngleHandling(angle, direction);

      printf("%ld\n", angularPosition);
    }
}

// Recursive error handling function
void errorHandling (int* errorCount){
    AS5047::T_AS5047_Error cr;
    short diagValue;

    *errorCount += 1;
    
    if (*errorCount > 20) {
        printf ("Too many errors - Aborting\n");
        while (true) {thread_sleep_for(WAIT_TIME_MS);}
    } else {
        cr = sensor.diagnose(&diagValue);
        if (cr != 0) errorHandling(errorCount);
    }
}

long incrementalAngleHandling (short angle, AS5047::T_AS5047_Rotation direction){
    static long lastAngle = 0;
    static bool firstTime = true;
    long diffAngle;

    if (firstTime){
        lastAngle = angle;
        diffAngle = 0;
        firstTime = false;
    } else {
        if ((direction == AS5047::CW_increasing) && (angle < lastAngle)) angle = angle + 16384;
        if ((direction == AS5047::CCW_decreasing) && (angle > lastAngle)) angle = angle - 16384;
        diffAngle = (long)angle - lastAngle;
        // Updating last angle value
        lastAngle = (long)angle;
    }
    return diffAngle;
}

