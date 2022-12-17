/**
 * @file AS5047.h
 * @brief Header file containing the AS5047 class compatible with mbed-os 6 (baremetal or multithread) using SPI interface all function are blocking
 * @author Hugues Angelis - Henoc Mukumbi
 * @date March 2022
 *
 * @section LICENSE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * AS5047 is a Hall effect on axis angular position sensor widely used in motion control systems. This library purpose is to configure (RAM only) the sensor and read the angular position using the SPI bus. No others communication interface are available in this library.
 * The AS5047P is a 14-Bit On-Axis Magnetic Rotary Position Sensor with 12-Bit Decimal and Binary Incremental Pulse Count with 28k rpm High Speed Capability
 * The AS5047U is a 14-Bit On-Axis Magnetic Rotary Position Sensor with Up to 14-Bit Binary Incremental Pulse Count
 * 
 * Datasheet can be found at AMS website : https://ams.com/en/ams-start
 *
 * AS5047P datasheet : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * AS5047U datasheet : https://ams.com/documents/20143/36005/AS5047U_DS000637_1-01.pdf
 * 
 * This library was written and tested by H. Angelis (IUT de CACHAN), please report bugs.
 * 
 * @warning The AS5047 class is intended for use of AS5047P ou AS5047U but not AS5047D (witch is not able to give angular position readout using SPI bus).
 *
 */

#ifndef _AS5047_H_
#define _AS5047_H_

#include "mbed.h"

#define AS5047_15BITMASK        0x7FFF
#define AS5047_14BITMASK        0x3FFF

#define AS5047_RES_4000         0
#define AS5047_RES_2000         1
#define AS5047_RES_1600         2
#define AS5047_RES_1200         3
#define AS5047_RES_800          4
#define AS5047_RES_400          5
#define AS5047_RES_200          6
#define AS5047_RES_100          7
#define AS5047_RES_4096         0
#define AS5047_RES_2048         1
#define AS5047_RES_1024         2

#define AS5047_ERR_OK           0
#define AS5047_ERR_PARITY       1
#define AS5047_ERR_FLAG         2
#define AS5047_ERR_ADDRESS      4
#define AS5047_ERR_DATA         8
#define AS5047_ERR_PARAM        16
#define AS5047_ERR_WRITE        32

#define AS5047_DIAG_OK          0
#define AS5047_DIAG_PARITY      1
#define AS5047_DIAG_INVCOMM     2
#define AS5047_DIAG_FRAME       4
#define AS5047_DIAG_MAGL        8
#define AS5047_DIAG_MAGH        16
#define AS5047_DIAG_CORDIC      32
#define AS5047_DIAG_NOT_READY   64
#define AS5047_DIAG_FAIL        128
#define AS5047_DIAG_ERR_FAIL    256

class AS5047 {

/**
 * \class AS5047 AS5047.h
 * AS5047 is a Hall effect on axis angular position sensor widely used in motion control systems. This class is intended to configure the sensor and read the angular position using the SPI bus. No others communication interface are available in this library
 * \brief More informations at : https://ams.com/en/ams-start
 * \note All function are blocking (ie : function returns only when data have been received and/or order processed), to avoid being stuck, use watchdogs.
 * \section EXAMPLE
 * The code below describe how to use a AS5047P sensor (without watchdogs).
 *
 * \code
 * #include "mbed.h"
 * #include "AS5047.h"
 * #include <chrono>
 * 
 * #define WAIT_TIME_MS 500
 * 
 * DigitalOut led1(LED1);
 * AS5047 sensor (SPI_MOSI, SPI_MISO, SPI_SCK, PF_14, 10000000);
 * Timer chronometer;
 * 
 * void errorHandling (int* errorCount);
 * long incrementalAngleHandling (short angle, AS5047::T_AS5047_Rotation direction);
 * 
 * int main()
 * {
 *     bool ready;
 *     int  errorCount = 0;
 *     AS5047::T_AS5047_Error cr;
 * 
 *     long angularPosition = 0;   // "long" is suitable for less than 131 072 turn else choose "long long". Avoid floating point numbers.
 *     short angle;
 *     AS5047::T_AS5047_Rotation direction;
 *     
 *     Watchdog &theDog = Watchdog::get_instance();
 *     theDog.start(10000);
 * 
 *     printf("This is the bare metal blinky example running on Mbed OS %d.%d.%d.\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);
 * 
 *     do {
 *         cr = sensor.isAS5047Ready(&ready);
 *     } while ((cr != 0) || (!ready));
 * 
 *     chronometer.start();
 *     chronometer.reset();
 * 
 *     while (true)
 *     {
 *       theDog.kick();
 *       while (chronometer.elapsed_time().count() < 1000); // 1ms wait between update
 *       chronometer.reset();
 * 
 *       if (errorCount > 0) errorCount--;
 * 
 *       cr = sensor.getAngularPosition(&angle); // get angle
 *       if (cr != AS5047_ERR_OK) errorHandling(&errorCount);
 * 
 *       cr = sensor.getDirection(&direction); // get direction
 *       if (cr != AS5047_ERR_OK) errorHandling(&errorCount);
 * 
 *       angularPosition += incrementalAngleHandling(angle, direction);
 * 
 *       printf("%ld\n", angularPosition);
 *     }
 * }
 * 
 * // Recursive error handling function
 * void errorHandling (int* errorCount){
 *     AS5047::T_AS5047_Error cr;
 *     short diagValue;
 * 
 *     *errorCount += 1;
 *     
 *     if (*errorCount > 20) {
 *         printf ("Too many errors - Aborting\n");
 *         while (true) {thread_sleep_for(WAIT_TIME_MS);}
 *     } else {
 *         cr = sensor.diagnose(&diagValue);
 *         if (cr != 0) errorHandling(errorCount);
 *     }
 * }
 * 
 * long incrementalAngleHandling (short angle, AS5047::T_AS5047_Rotation direction){
 *     static long lastAngle = 0;
 *     static bool firstTime = true;
 *     long diffAngle;
 * 
 *     if (firstTime){
 *         lastAngle = angle;
 *         diffAngle = 0;
 *         firstTime = false;
 *     } else {
 *         if ((direction == AS5047::CW_increasing) && (angle < lastAngle)) angle = angle + 16384;
 *         if ((direction == AS5047::CCW_decreasing) && (angle > lastAngle)) angle = angle - 16384;
 *         lastAngle = (long)angle;
 *         diffAngle = (long)angle - lastAngle;
 *     }
 *     return diffAngle;
 * }
 * \endcode
 */

private :

/**
 *  \brief short hand for unsigned short.
 *  \note Value from 0 to 65535.
 */
typedef unsigned short ushort;

/**
 *  \brief short hand for unsigned char.
 *  \note Value from 0 to 255.
 */
typedef unsigned char uchar;

public :

/**
 *  Easy to use type to identify rotation direction.
 *  \brief Explicit information list :
 *  \param CW_increasing        = 0   : magnet is turning Clockwise (ie : angle value is increasing)
 *  \param CCW_decreasing       = 1   : magnet is turning Counterclockwise (ie : angle value is decreasing)
 */
typedef enum {
    CW_increasing  = 0,
    CCW_decreasing = 1
} T_AS5047_Rotation;

/**
 *  Error(s) are detected by the functions and indicate a problem coming from code execution.
 *  \brief Explicit error code list :
 *  \param AS5047_ERR_OK      = 0   : OK - No error
 *  \param AS5047_ERR_PARITY  = 1   : Parity Error - parity compute by the function is not the one present in the received frame
 *  \param AS5047_ERR_FLAG    = 2   : Error Flag - The sensor has detected a problem with the last frame he has received or is experiencing a malfunction.
 *  \param AS5047_ERR_ADDRESS = 4   : Address Error - Address given to the function is not a valid register address
 *  \param AS5047_ERR_DATA    = 8   : Data Error - Data given to the function is not of the good format
 *  \param AS5047_ERR_PARAM   = 16  : Parameter Error - The parameter given to the function is not valid
 *  \param AS5047_ERR_WRITE   = 32  : Write Order Fail - The write command fail to change the value of the register
 *  \note The error value is the sum of all error detected (for example if there is a parity error and the error flag is raised, return code will be 3 (= 1 + 2))
 *  ERROR_FLAG is a flag set by the sensor to indicate a problem (see diagnostic section).
 */
typedef short T_AS5047_Error;

/**
 *  \brief Explicit diagnostic code list :
 *  \param AS5047_DIAG_OK        =   0 : OK - No error
 *  \param AS5047_DIAG_PARITY    =   1 : Parity Error - The parity computed by the sensor is different of the one in the frame it received
 *  \param AS5047_DIAG_INVCOMM   =   2 : Invalid command - The command address is not valid
 *  \param AS5047_DIAG_FRAME     =   4 : Frame Error - The frame received is not a valid SPI frame
 *  \param AS5047_DIAG_MAGL      =   8 : Magnetic error - The magnetic field is too low and sensor is not able to give a reliable position vector 
 *  \param AS5047_DIAG_MAGH      =  16 : Magnetic error - The magnetic field is too strong and sensor is not able to give a reliable position vector 
 *  \param AS5047_DIAG_CORDIC    =  32 : CORDIC Error - The coordinate rotating digital computer is experiencing an overflow and sensor is not able to give a reliable position vector
 *  \param AS5047_DIAG_NOT_READY =  64 : NOT READY - The offset compensation loop hasn't finish converging and sensor is giving an unaccurate position vector
 *  \param AS5047_DIAG_FAIL      = 128 : Diagnostic frame error - The diagnostic frame was erroneous
 *  \param AS5047_DIAG_ERR_FAIL  = 256 : Error Flag Frame Error - The error flag frame was erroneous
 *  \note The value obtained is the sum of all detected errors (for example if there is a NOT READY and a MAGH, value will be 80 (= 16 + 64))
 *  \note The value of diagnostic is obtained by a sensor interrogation and list errors détected by the sensor.
 */
typedef short T_AS5047_DIAG;

/**
 *  \brief Explicit registers address code list :
 *  \param AS5047_ADR_NOP        = 0x0000 : No Operation Register - Read Only
 *  \param AS5047_ADR_ERRFL      = 0x0001 : Error Flag Register - Read Only
 *  \param AS5047_ADR_PROG       = 0x0003 : Programming Register - Read/Write
 *  \param AS5047_ADR_ZPOSM      = 0x0016 : Zero Position MSB Register - Read/Write/Program
 *  \param AS5047_ADR_ZPOSL      = 0x0017 : Zero Position LSB Register - Read/Write/Program
 *  \param AS5047_ADR_SETTING1   = 0x0018 : #1 Setting Register - Read/Write/Program
 *  \param AS5047_ADR_SETTING2   = 0x0019 : #2 Setting Register - Read/Write/Program
 *  \param AS5047_ADR_DIAAGC     = 0x3FFC : Diagnostic & Automatic Gain control Register - Read Only
 *  \param AS5047_ADR_MAG        = 0x3FFD : Magnetic Field value Register - Read Only
 *  \param AS5047_ADR_ANGLE      = 0x3FFE : Raw Angle value Register - Read Only
 *  \param AS5047_ADR_ANGLECOM   = 0x3FFF : DAE Compensated Angle value Register - Read Only
 *  \note The AS5047 has a OTP (one time programming) memory, therefore programming has not been implemented. Writing in those register stores data in a non volatile memory (that remains until shutdown).
 */
typedef unsigned short T_AS5047_ADR;

/*********************** GLOBAL VAR ***********************/

/**
 * @var T_AS5047_DIAG AS5047_diagnostic
 * @brief diagnostic value (see T_AS5047_DIAG for detail). Those value are automaticaly updated each time the Error Flag is set. 
 */
T_AS5047_DIAG AS5047_diagnostic;


/*********************** METHODS ***********************/

/**
 * @brief Constructor of AS5047 object.
 * @param mosi             : the Mbed pin used as MOSI  (Master Out - Slave In)
 * @param miso             : the Mbed pin used as MISO  (Master In - Slave Out)
 * @param sck              : the Mbed pin used as SCK   (serial clock)
 * @param ss               : the Mbed pin used as SS    (Slave Select)
 * @param freq             : the serial clock frequency (default is 1MHz or 1Mbits/s)
 * @param invertBytesOrder : swap the lower and upper byte (default is Yes)
 * @note The SS pin can be any digital output and has no obligation of being hardware associated with the SPI bus
 * @note We used the 8 bit SPI mode (because 16 bit doesn't look functionnal), so, we need to swap bytes :
 *  - SPI bus is configured in Big Endian (bits are transmitted in the order : MSB to LSB).
 *  - ARM is Little Endian so it sends word's bytes from LSB to MSB (LSB is sent first).
 *  - So a 16 bits word is sent : bit 7 - ... - bit 0 (first byte), then : bit 15 - ... - bit 8 (second byte)
 *  - AS5047 SPI interface requires bits sent msb first. So we need to swap bytes before transmition and after reception.
 *  - You may change invertBytesOrder to false, if you use 16 bits SPI mode.
 */
AS5047 (PinName mosi, PinName miso, PinName sck, PinName ss, int freq = 1000000, bool invertBytesOrder = true);

/**
 * Destructor of AS5047 object.
 */
~AS5047 ();

/**
 * @brief Set how many step per turns are provided by the ABI interface of the sensor.
 * @note Possible values for resolution are :
 *  - 4096 (4096 steps/turn or 1024 pulse/channel/turn)
 *  - 4000 (4000 steps/turn or 1000 pulse/channel/turn)
 *  - 2048 (2048 steps/turn or  512 pulse/channel/turn)
 *  - 2000 (2000 steps/turn or  500 pulse/channel/turn)
 *  - 1600 (1600 steps/turn or  400 pulse/channel/turn)
 *  - 1200 (1200 steps/turn or  300 pulse/channel/turn)
 *  - 1024 (1024 steps/turn or  256 pulse/channel/turn)
 *  - 800  ( 800 steps/turn or  200 pulse/channel/turn)
 *  - 400  ( 400 steps/turn or  100 pulse/channel/turn)
 *  - 200  ( 200 steps/turn or   50 pulse/channel/turn)
 *  - 100  ( 100 steps/turn or   25 pulse/channel/turn)
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param resolution (short, passed by value) : resolution of the sensor.
 * @return T_AS5047_Error : error code.
 * @note Invalid resolution command will return a AS5047_ERR_PARAM error.
 */
T_AS5047_Error setResolution (short resolution);

/**
 * @brief identify how many step per turns are provided by the ABI interface of the sensor.
 * @note Possible return values are :
 *  - 4096 (4096 steps/turn or 1024 pulse/channel/turn)
 *  - 4000 (4000 steps/turn or 1000 pulse/channel/turn)
 *  - 2048 (2048 steps/turn or  512 pulse/channel/turn)
 *  - 2000 (2000 steps/turn or  500 pulse/channel/turn)
 *  - 1600 (1600 steps/turn or  400 pulse/channel/turn)
 *  - 1200 (1200 steps/turn or  300 pulse/channel/turn)
 *  - 1024 (1024 steps/turn or  256 pulse/channel/turn)
 *  - 800  ( 800 steps/turn or  200 pulse/channel/turn)
 *  - 400  ( 400 steps/turn or  100 pulse/channel/turn)
 *  - 200  ( 200 steps/turn or   50 pulse/channel/turn)
 *  - 100  ( 100 steps/turn or   25 pulse/channel/turn)
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param resolution (short, passed by address) : resolution of the sensor.
 * @return T_AS5047_Error : error code.
 */
T_AS5047_Error getResolution (short *resolution);

/**
 * @brief Identify the cause of an error when error flags are raised.
 * @note Possible returned value is the sum of :
 *  - 0  : No error detected
 *  - 1  : Parity error            - The parity bit in the frame is not the same that the one that has been computed.
 *  - 2  : Invalid command         - The last command has not been understood by the sensor.
 *  - 4  : Invalid frame           - The frame does not correspond to sensor's SPI format.
 *  - 8  : Magnetic field too low  - The magnetic field is too low to get a good angular position.
 *  - 16 : Magnetic field too high - The magnetic field is too high to get a good angular position.
 *  - 32 : CORDIC error            - The sensor coordinate computer has failed to compute an accurate position.
 *  - 64 : Not ready               - The sensor is not ready.
 *  - 128 : DIAG Error             - The reading of DIAG frame has failed
 *  - 256 : Error Flag Error       - The reading of the Error Flag has failed
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param diagValue (T_AS5047_DIAG, passed by address) : sum of active codes - Not mandatory (one may use the global variable : AS5047_diagnostic)
 * @return T_AS5047_Error : error code.
 */
T_AS5047_Error diagnose (T_AS5047_DIAG *diagValue);

/**
 * @brief Allow the sensor to configurate the number of pair poles of a brushless motor between 1 and 8 pair poles.
 * @note The number of pair poles is dependent of the motor, it should be configured to allow the sensor to replace classical Hall Sensors and configured to allow the motor driver to work accordingly with the motor.
 * The number of pair poles is the given value plus one and the number of poles is twice the number of pair poles.
 * So setting pairPoles to 0 means 1 pair pole (or 2 poles), setting it to 4 means 5 pair poles (or 10 poles).
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param pairPoles (short, passed by value) : number of pair poles minus 1
 * @return T_AS5047_Error : error code.
 */
T_AS5047_Error setUVWPairPoles (short pairPoles);

/**
 * @brief Activate the PWM output on the selected pin (0 for W pin, 1 for I pin).
 * @note The PWM signal is based on a 444 ns clock with a period of 4119 clock cycle (12 + 4 + 4095 + 8 period of the clock).
 *  The signal is composed of 12 periods high, followed by 4 periods low and then between 0 and 4095 period for the angular position, and then 8 periods low.
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param port (int, passed by value) : selected output pin (0 for W pin used as PWM output, 1 for I pin used as PWM output).
 * @return T_AS5047_Error : error code.
 * @note Depending on the selected output pin, it diasble UVW or ABI output (0 will disable the UVW output function, 1 will disable the ABI output function).
 */
T_AS5047_Error ActivatePWM (int port);

/**
 * @brief Get the direction of the rotation as a value : 
 * - CW_increasing = 0   : clockwise - angle is increasing,
 * - CCW_decreasing = 1 : counterclockwise - angle is decreasing.
 * @note Direction is the instant value of the spinning direction. As position value is absolute (not incremental), you may use this function for both overflow or underflow of the position - ie : when position reach one of the limits of the span (-8192 to 8191) - or any other reason.  
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param direction (int, passed by address) : spinning direction when the order has been processed.
 * @return T_AS5047_Error : error code.
 */
T_AS5047_Error getDirection (T_AS5047_Rotation *direction);

/**
 * @brief Get readiness information about the convergence of the Kalman's filter of the sensor.
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param ready (boolean, passed by address) : true when calibration algorithm has converged, false otherwise.
 * @return T_AS5047_Error : error code.
 */
T_AS5047_Error isAS5047Ready (bool *ready);

/**
 * @brief Get the absolute angle position of the magnet relative to the sensor in 14 bits signed value.
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param position (short, passed by address) : 14 bits angle value (from -8192 up to 8192).
 * @return T_AS5047_Error : error code.
 */
T_AS5047_Error getAngularPosition (short *position);

/**
 * @brief Get the absolute angle with dynamic error compensation position of the magnet relative to the sensor in 14 bits signed value.
 * @note Position returned by this method is not suitable in most case (only constant rotation speed can bring to a valid result)   
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param position (short, passed by address) : 14 bits angle value (from -8192 up to 8192).
 * @return T_AS5047_Error : error code.
 */
T_AS5047_Error getDAECPosition (short *position);

/**
 * @brief Write a data value in a specific register of the sensor.
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param registerAddress (T_AS5047_ADR, passed by value) : 14 bits register address value.
 * @param registerValue   (short, passed by value)        : 14 bits register data value.
 * @return T_AS5047_Error : error code.
 */
T_AS5047_Error writeToSensor (T_AS5047_ADR registerAddress, short registerValue);

/**
 * @brief Read the data value of a specific register of the sensor.
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param registerAddress (T_AS5047_ADR, passed by value) : 14 bits register address value.
 * @param registerValue   (short, passed by address)      : 14 bits register data value.
 * @return T_AS5047_Error : error code.
 */
T_AS5047_Error readFromSensor (T_AS5047_ADR registerAddress, short* registerValue);

private :

#define AS5047_ADR_NOP          0x0000
#define AS5047_ADR_ERRFL        0x0001
#define AS5047_ADR_PROG         0x0003
#define AS5047_ADR_ZPOSM        0x0016
#define AS5047_ADR_ZPOSL        0x0017
#define AS5047_ADR_SETTING1     0x0018
#define AS5047_ADR_SETTING2     0x0019
#define AS5047_ADR_DIAAGC       0x3FFC
#define AS5047_ADR_MAG          0x3FFD
#define AS5047_ADR_ANGLE        0x3FFE
#define AS5047_ADR_ANGLECOM     0x3FFF

/************************ OBJECTS ************************/
/**
 * @obj SPI _sensor
 * @brief SPI bus used to communicate. 
 */
SPI* _sensor;

/*********************** FRAMES ***********************/

/**
 *  \struct T_bits
 *  \brief  bitfield that matches frame format of a SPI's AS5047 frame type. 
 *  \note The bitfield is composed of :
 *  \param  data   (unsigned short) : Frame payload (14 bits) - bits from 0 to 13
 *  \param  flag   (unsigned short) : Frame flag bit (1 bit) - bit 14 of the frame, used as R/W for command frame, and Error Flag for read data frame. Must be set to 0 for write data frame 
 *  \param  parity (unsigned short) : Frame parity bit (1 bit) - bit 15 of the frame, even parity calculated on the lower 15 bits of the frame
 *  @note More info can be found here : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf (page 12)
 */
typedef struct {
    ushort data      :14;
    ushort flag      :1;
    ushort parity    :1;
} T_bits; 

/**
 *  \union T_packet
 *  \brief  union of 3 objects (objects overlap) to match the frame format of a SPI's AS5047 frame type 
 *  \note This structure is the union of : a 16 bits word (word), a table of 2 bytes (bytes[2]) and a bitfield (Bits), allowing user to access the frame as a short, 2 bytes or frame fields.
 *  \param  Bits   (16 bits bitfield) : The 16 frame's bits (see below for description, bit 0 is the less significant bit)
 *  \param  bytes  (char [2])         : Table of 2 bytes (16 bits) - used for byte access to the frame (bytes[0] is the less significant byte) 
 *  \param  word   (unsigned short)   : Single word value (16 bits) - used for word access to the frame
 *  @note This structure is a multipupose tool allowing user to access the frame bitwise or bytewise or wordwise 
 *  @note More info can be found here : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf (page 12)
 */
typedef union {
    ushort      word;
    char        bytes[2];
    T_bits      Bits;
}T_packet;

/*********************** AS5047 REGISTERS ***********************/

/**
 *  \struct T_ERRFL_REG
 *  \brief  bitfield that matches ERRFL register (address 0x0001) of the AS5047. 
 *  \note The bitfield is composed of :
 *  \param  FRERR   (unsigned short) : Frame error bit (bit 0) - SPI frame is not valid - true if set
 *  \param  INVCOMM (unsigned short) : Invalid command bit (bit 1) - SPI Address is not a valid AS5047P register  - true if set
 *  \param  PARERR  (unsigned short) : Parity error bit (bit 2) - Previous frame parity was erroneous - true if set 
 *  @note More info can be found here : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf (page 15)
 */
/**
 *  \union T_ERRFL_REG
 *  \brief  union of a T_packet to match the frame format of a SPI's AS5047 communication and the ERRFL Register (address = 0x0001) bit field of the AS5047 objects (objects overlap)
 *  \note This structure is the union of : a 16 bits word (unsigned short), a table of 2 bytes (bytes[2]) and a bitfield (Bits), allowing user to access the frame as a short, 2 bytes or frame fields.
 */
typedef union {
    T_packet    packet;
    struct {
        ushort   FRERR   : 1;
        ushort   INVCOMM : 1;
        ushort   PARERR  : 1;
        ushort           : 13;
    }Bits;
}T_ERRFL_REG;

/**
 *  \struct T_PROG_REG
 *  \brief  bitfield that matches PROG register (address 0x0003) of the AS5047. 
 *  \note The bitfield is composed of :
 *  \param  PROGEN  (unsigned short) : Program OTP Enable bit (bit 0) - Enables programming the entire OTP memory - true if set
 *  \param  OTPREF  (unsigned short) : Refresh RAM bit (bit 2) - Refreshes the non-volatile memory content with the OTP programmed content - true if set
 *  \param  PROGOTP (unsigned short) : Start Programming bit (bit 3) - Start OTP programming cycle - true if set
 *  \param  PROGVER (unsigned short) : Program Verify bit (bit 6) - Must be set to 1 for verifying the correctness of the OTP programming - true if set 
 *  @note More info can be found here : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf (page 16)
 */
/**
 *  \union T_PROG_REG
 *  \brief  union of a T_packet to match the frame format of a SPI's AS5047 communication and the PROG Register (address = 0x0003) bit field of the AS5047 objects (objects overlap)
 *  \note This structure is the union of : a 16 bits word (unsigned short), a table of 2 bytes (bytes[2]) and a bitfield (Bits), allowing user to access the frame as a short, 2 bytes or frame fields.
 */
typedef union {
    T_packet    packet;
    struct {
        ushort   PROGEN  : 1;
        ushort           : 1;
        ushort   OTPREF  : 1;
        ushort   PROGOTP : 1;
        ushort           : 2;
        ushort   PROGVER : 1;
        ushort           : 9;
    }Bits;
}T_PROG_REG;

/**
 *  \struct T_DIAAGC_REG
 *  \brief  bitfield that matches DIAAGC register (address 0x3FFC) of the AS5047. 
 *  \note The bitfield is composed of :
 *  \param  AGC     (unsigned short) : Automatic Gain Control value (bit 7-0) - Automatic Gain Control value - Read Only
 *  \param  LF      (unsigned short) : Loop Finish bit (bit 8) - Set to 1 when Internal Offset loop is finished - Read Only
 *  \param  COF     (unsigned short) : CORDIC Overflow bit (bit 9) - Set to 1 when the measured angle is not reliable - Read Only
 *  \param  MAGH    (unsigned short) : Magnetic Field too high bit (bit 10) - Set to 1 when AGC = 0 - Read Only 
 *  \param  MAGL    (unsigned short) : Magnetic Field too Low bit (bit 11) - Set to 1 when AGC = 0xFF - Read Only 
 *  @note More info can be found here : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf (page 16)
 */
/**
 *  \union T_DIAAGC_REG
 *  \brief  union of a T_packet to match the frame format of a SPI's AS5047 communication and the DIAAGC Register (address = 0x3FFC) bit field of the AS5047 objects (objects overlap)
 *  \note This structure is the union of : a 16 bits word (unsigned short), a table of 2 bytes (bytes[2]) and a bitfield (Bits), allowing user to access the frame as a short, 2 bytes or frame fields.
 */
typedef union {
    T_packet    packet;
    struct {
        ushort   AGC     : 8;
        ushort   LF      : 1;
        ushort   COF     : 1;
        ushort   MAGH    : 1;
        ushort   MAGL    : 1;
        ushort           : 4;
    }Bits;
}T_DIAAGC_REG;

/**
 *  \struct T_ZPOSM_REG
 *  \brief  bitfield that matches ZPOSM register (address 0x0016) of the AS5047. 
 *  \note The bitfield is composed of :
 *  \param  ZPOSM   (unsigned short) : Zero Position MSB value (bit 7-0) - 8 most significant bits of the zero position
 *  @note More info can be found here : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf (page 17)
 */
/**
 *  \union T_ZPOSM_REG
 *  \brief  union of a T_packet to match the frame format of a SPI's AS5047 communication and the ZPOSM Register (address = 0x0016) bit field of the AS5047 objects (objects overlap)
 *  \note This structure is the union of : a 16 bits word (unsigned short), a table of 2 bytes (bytes[2]) and a bitfield (Bits), allowing user to access the frame as a short, 2 bytes or frame fields.
 */
typedef union {
    T_packet    packet;
    struct {
        ushort   ZPOSM   : 8;
        ushort           : 8;
    }Bits;
}T_ZPOSM_REG;

/**
 *  \struct T_ZPOSL_REG
 *  \brief  bitfield that matches ZPOSL register (address 0x0017) of the AS5047. 
 *  \note The bitfield is composed of :
 *  \param  ZPOSL   (unsigned short) : Zero Position LSB value (bit 5-0) - 6 less significant bits of the zero position
 *  \param  COMP_L  (unsigned short) : MAGH Contribute (bit 6) - This bit enables the contribution of MAGH (Magnetic field strength too high) to the error flag
 *  \param  COMP_H  (unsigned short) : MAGL Contribute (bit 7) - This bit enables the contribution of MAGL (Magnetic field strength too low) to the error flag
 *  @note More info can be found here : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf (page 18)
 */
/**
 *  \union T_ZPOSL_REG
 *  \brief  union of a T_packet to match the frame format of a SPI's AS5047 communication and the ZPOSL Register (address = 0x0017) bit field of the AS5047 objects (objects overlap)
 *  \note This structure is the union of : a 16 bits word (unsigned short), a table of 2 bytes (bytes[2]) and a bitfield (Bits), allowing user to access the frame as a short, 2 bytes or frame fields.
 */
typedef union {
    T_packet    packet;
    struct {
        ushort   ZPOSL   : 6;
        ushort   COMP_L  : 1;
        ushort   COMP_H  : 1;
        ushort           : 8;
    }Bits;
}T_ZPOSL_REG;

/**
 *  \struct T_SETTING1_REG
 *  \brief  bitfield that matches SETTING1 register (address 0x0018) of the AS5047. 
 *  \note The bitfield is composed of :
 *  \param  FSET    (unsigned short) : Factory setting (bit 0) - bit pre-programmed to 1 - Read Only
 *  \param  UNUSED  (unsigned short) : Not used (bit 1) - bit pre-programmed to 0 - Must not be overwritten
 *  \param  DIR     (unsigned short) : Direction (bit 2) - direction bit, 0 for CW and 1 for CCW, writing 1 invert both value
 *  \param  UVWABI  (unsigned short) : PWM output Pin (bit 3) - Defines the PWM Output (0 = ABI is operating, W is used as PWM, 1 = UVW is operating, I is used as PWM)
 *  \param  DAECDIS (unsigned short) : Disable DAEC (bit 4) - Disable Dynamic Angle Error Compensation (0 = DAE compensation ON, 1 = DAE compensation OFF)
 *  \param  ABIBIN  (unsigned short) : ABI value type (bit 5) - ABI decimal (1000-500-400-300-200-100) or binary (1024-512-256) selection of the ABI pulses per revolution
 *  \param  DATASEL (unsigned short) : DataSelect (bit 6) - This bit defines which data can be read form address 16383dec (3FFFhex). 0->DAECANG, 1->CORDICANG
 *  \param  PWMON   (unsigned short) : PWMon (bit 7) - Enables PWM (setting of UVW_ABI Bit necessary)
 *  @note More info can be found here : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf (page 18)
 */
/**
 *  \union T_SETTING1_REG
 *  \brief  union of a T_packet to match the frame format of a SPI's AS5047 communication and the SETTING1 Register (address = 0x0018) bit field of the AS5047 objects (objects overlap)
 *  \note This structure is the union of : a 16 bits word (unsigned short), a table of 2 bytes (bytes[2]) and a bitfield (Bits), allowing user to access the frame as a short, 2 bytes or frame fields.
 */
typedef union {
    T_packet    packet;
    struct {
        ushort   FSET    : 1;
        ushort   UNUSED  : 1;
        ushort   DIR     : 1;
        ushort   UVW_ABI : 1;
        ushort   DAECDIS : 1;
        ushort   ABIBIN  : 1;
        ushort   DATASEL : 1;
        ushort   PWMON   : 1;
        ushort           : 8;
    }Bits;
}T_SETTING1_REG;

/**
 *  \struct T_SETTING2_REG
 *  \brief  bitfield that matches SETTING2 register (address 0x0019) of the AS5047. 
 *  \note The bitfield is composed of :
 *  \param  UVWPP   (unsigned short) : UVW Pair Pole (bit 2-0) - UVW number of pole pairs (000 = 1, 001 = 2, 010 = 3, 011 = 4, 100 = 5, 101 = 6, 110 = 7, 111 = 7)
 *  \param  HYS     (unsigned short) : Hysteresis (bit 4-3) - Hysteresis setting (from 0 to 3 LSB)
 *  \param  ABIRES  (unsigned short) : ABI Resolution (bit 7-5) - Résolution of ABI
 *  @note More info can be found here : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf (page 19)
 */
/**
 *  \union T_SETTING2_REG
 *  \brief  union of a T_packet to match the frame format of a SPI's AS5047 communication and the SETTING2 Register (address = 0x0019) bit field of the AS5047 objects (objects overlap)
 *  \note This structure is the union of : a 16 bits word (unsigned short), a table of 2 bytes (bytes[2]) and a bitfield (Bits), allowing user to access the frame as a short, 2 bytes or frame fields.
 */
typedef union {
    T_packet    packet;
    struct {
        ushort   UVWPP   : 3;
        ushort   HYS     : 2;
        ushort   ABIRES  : 3;
        ushort           : 8;
    }Bits;
}T_SETTING2_REG;

typedef enum {
    dataWrite = 0,
    dataRead = 1
} T_dataDirection;

/**
 * Compute even parity of the data.
 * @brief Compute the value of the even parity of the given data.
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf (page 13).
 * @param value (unsigned short, passed by value) : 16 bits value.
 * @return int : The even parity (1 or 0).
 */
int ComputeEvenParity (ushort value);

/**
 * Check the validity of a register address.
 * @brief Check if the value of the given address is a valid register number.
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param address (unsigned short, passed by value) : address value to test.
 * @return bool : The validity of the address (true or false).
 */
bool isAddressValid (T_AS5047_ADR address);

/**
 * Check the validity of a data value.
 * @brief Check if the value of the given data is valid (value between 0 and 16383 bits).
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param value (unsigned short, passed by value) : value to test.
 * @return bool : The validity of the data (true or false).
 */
bool isDataValid (ushort value);

/**
 * Change the byte order of a 16 bits word.
 * @brief Change the byte order of a 16 bits word value.
 * @param value (unsigned short, passed by value) : 16 bits value.
 * @return unsigned short : byte order inverted value.
 */
ushort swapBytes (ushort value);

/**
 * Check the validity of a frame.
 * @brief Check the validity of the received frame by computing even parity and checking the Error Flag bit
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param packet (T_packet, passed by value) : The received frame.
 * @return T_AS5047_Error : error code.
 */
T_AS5047_Error isPaquetValid (T_packet packet);

/**
 * Create a valid frame.
 * @brief Create a valid frame by assembling fields and computing Even parity
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param packet    (T_packet, passed by address)      : The packet to be produce.
 * @param value     (unsigned short, passed by value)  : The packet's 14 bits value.
 * @param direction (T_dataDirection, passed by value) : The data direction (Read or write) of the packet.
 * @return T_AS5047_Error : error code.
 */
T_AS5047_Error createFrame (T_packet *paquet, ushort value, T_dataDirection direction = dataWrite);

/**
 * Write a data to a register
 * @brief Create a series of valid frame to write a 14 bits value in a register
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param address (unsigned short, passed by value) : The 14 bits register address.
 * @param value   (unsigned short, passed by value) : The 14 bits register value.
 * @return T_AS5047_Error : error code.
 */
T_AS5047_Error writeData (T_AS5047_ADR address, ushort value);

/**
 * Read the data in a register
 * @brief Create a series of valid frame to read a 14 bits value in a register
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param address (unsigned short, passed by value)   : The 14 bits register address.
 * @param value   (unsigned short, passed by address) : The 14 bits register value.
 * @return T_AS5047_Error : error code.
 */
T_AS5047_Error readData (T_AS5047_ADR address, ushort *value);

/*********************** GLOBAL VAR ***********************/

/**
 * @var bool _byteSwapping
 * @brief identify the need to swap bytes in a 16 bit word. 
 */
bool _byteSwapping;

};
#endif