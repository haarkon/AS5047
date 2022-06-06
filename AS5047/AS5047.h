/**
 * @file AS5047.h
 * @brief Header file containing the AS5047 class compatible with mbed-os 6 (baremetal or multithread) using SPI interface all function are blocking
 * @author Hugues Angelis
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
#define AS5047_ERR_FAIL         32

#define AS5047_DIAG_OK          0
#define AS5047_DIAG_PARITY      1
#define AS5047_DIAG_INVCOMM     2
#define AS5047_DIAG_FRAME       4
#define AS5047_DIAG_MAGL        8
#define AS5047_DIAG_MAGH        16
#define AS5047_DIAG_CORDIC      32
#define AS5047_DIAG_NOT_READY   64

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
 * 
 * #define WAIT_TIME_MS 100ms 
 * DigitalOut led1(LED1);
 * AS5047 sensor (MISO, MOSI, SCK, SS, 1000000);
 * 
 * int main()
 * {
 * }
 * \endcode
 */

public :

/**
 *  \typedef T_AS5047_Error
 *  Error(s) are detected by the functions and indicate a problem coming from code execution.
 *  \brief Explicit error code list :
 *  \param AS5047_ERR_OK      = 0   : OK - No error
 *  \param AS5047_ERR_PARITY  = 1   : Parity Error - parity compute by the function is not the one present in the received frame
 *  \param AS5047_ERR_FLAG    = 2   : Error Flag - The sensor has detected a problem with the last frame he has received or is experiencing a malfunction.
 *  \param AS5047_ERR_ADDRESS = 4   : Address Error - Address given to the function is not a valid register address
 *  \param AS5047_ERR_DATA    = 8   : Data Error - Data given to the function is not of the good format
 *  \param AS5047_ERR_PARAM   = 16  : Parameter Error - The parameter given to the function is not valid
 *  \note The error value is the sum of all error detected (for example if there is a parity error and the error flag is raised, return code will be 3 (= 1 + 2))
 *  ERROR_FLAG is a flag set by the sensor to indicate a problem (see diagnostic section).
 */
typedef short T_AS5047_Error;

/**
 *  \typedef T_AS5047_DIAG
 *  Error(s) that are detected by the sensor.
 *  The value of diagnostic is obtained by a sensor interrogation (that is automaticaly performed when the ERROR FLAG is set) and list errors détected by the sensor.
 *  \brief Explicit diagnostic code list :
 *  \param AS5047_DIAG_OK        = 0     : OK - No error
 *  \param AS5047_DIAG_PARITY    = 1     : Parity Error - The parity computed by the sensor is different of the one in the frame it received
 *  \param AS5047_DIAG_INVCOMM   = 2     : Invalid command - The command address is not valid
 *  \param AS5047_DIAG_FRAME     = 4     : Frame Error - The frame received is not a valid SPI frame
 *  \param AS5047_DIAG_MAGL      = 8     : Magnetic error - The magnetic field is too low and sensor is not able to give a reliable position vector 
 *  \param AS5047_DIAG_MAGH      = 16    : Magnetic error - The magnetic field is too strong and sensor is not able to give a reliable position vector 
 *  \param AS5047_DIAG_CORDIC    = 32    : CORDIC Error - The coordinate rotating digital computer is experiencing an overflow and sensor is not able to give a reliable position vector
 *  \param AS5047_DIAG_NOT_READY = 64    : NOT READY - The offset compensation loop hasn't finish converging and sensor is giving an unaccurate position vector
 *  \note The value obtained is the sum of all errors detected (for example if there is a NOT READY and a MAGH, value will be 80 (= 16 + 64))
 */
typedef short T_AS5047_DIAG;

/*********************** GLOBAL VAR ***********************/

/**
 * @var T_AS5047_DIAG AS5047_diagnostic
 * @brief diagnostic value (see T_AS5047_DIAG for detail). Those value are automaticaly updated each time the Error Flag is set. 
 */
T_AS5047_DIAG AS5047_diagnostic;


/*********************** METHODS ***********************/

/**
 * Constructor of AS5047 object.
 *
 * @param mosi      : the Mbed pin used as MOSI  (Master Out - Slave In)
 * @param miso      : the Mbed pin used as MISO  (Master In - Slave Out)
 * @param sck       : the Mbed pin used as SCK   (serial clock)
 * @param ss        : the Mbed pin used as SS    (Slave Select)
 * @param frequency : the serial clock frequency (default is 1MHz or 1Mbits/s)
 */
AS5047 (PinName mosi, PinName miso, PinName sck, PinName ss, int frequency = 1000000);

/**
 * Destructor of AS5047 object.
 */
~AS5047 ();


/**
 * Set the resolution of the sensor
 * @brief Define how many step per turns are provided by the sensor
 * @note Possible values are :
 *  4096 (4096 steps/turn or 1024 pulse/channel/turn)
 *  4000 (4000 steps/turn or 1000 pulse/channel/turn)
 *  2048 (2048 steps/turn or  512 pulse/channel/turn)
 *  2000 (2000 steps/turn or  500 pulse/channel/turn)
 *  1600 (1600 steps/turn or  400 pulse/channel/turn)
 *  1200 (1200 steps/turn or  300 pulse/channel/turn)
 *  1024 (1024 steps/turn or  256 pulse/channel/turn)
 *  800  ( 800 steps/turn or  200 pulse/channel/turn)
 *  400  ( 400 steps/turn or  100 pulse/channel/turn)
 *  200  ( 200 steps/turn or   50 pulse/channel/turn)
 *  100  ( 100 steps/turn or   25 pulse/channel/turn)
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param resolution (short, passed by value) : resolution of the sensor
 * @return T_AS5047_Error : error code.
 * @note non valid resolution command will return a AS5047_ERR_PARAM error
 */
T_AS5047_Error setResolution (short resolution);

/**
 * Set the resolution of the sensor
 * @brief Define how many step per turns are provided by the sensor
 * @note Possible values are :
 * @note More informations at : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf
 * @param resolution (short, passed by value) : resolution of the sensor
 * @return T_AS5047_Error : error code.
 * @note non valid resolution command will return a AS5047_ERR_PARAM error
 */
T_AS5047_Error diagnose (T_AS5047_DIAG *value);
T_AS5047_Error setUVWPairPoles (short pairPoles);
T_AS5047_Error ActivatePWM (int port);
T_AS5047_Error getDirection (int *direction);
T_AS5047_Error isAS5047Ready (bool *ready);
T_AS5047_Error getAngularPosition (short *position);

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
#define AS5047_ADR_ANGLEUNC     0x3FFE
#define AS5047_ADR_ANGLECOM     0x3FFF


/*********************** FRAMES ***********************/

/**
 *  \struct T_bits
 *  \brief  bitfield that matches frame format of a SPI's AS5047 frame type. 
 *  \note The bitfield is composed of :
 *  \param  data   (short) : Frame payload (14 bits) - bits from 0 to 13
 *  \param  flag   (short) : Frame flag bit (1 bit) - bit 14 of the frame, used as R/W for command frame, and Error Flag for read data frame. Must be set to 0 for write data frame 
 *  \param  parity (short) : Frame parity bit (1 bit) - bit 15 of the frame, even parity calculated on the lower 15 bits of the frame
 *  @note More info can be found here : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf (page 12)
 */
typedef struct {
    short data      :14;
    short flag      :1;
    short parity    :1;
} T_bits; 

/**
 *  \union T_packet
 *  \brief  union of 3 objects (objects overlap) to match the frame format of a SPI's AS5047 frame type 
 *  \note This structure is the union of : a 16 bits word (word), a table of 2 bytes (bytes[2]) and a bitfield (Bits), allowing user to access the frame as a short, 2 bytes or frame fields.
 *  \param  Bits   (short) : Bitfield (16 bits) - The 16 frame's bits (see below for description, bit 0 is the less significant bit)
 *  \param  bytes  (char)  : Table of 2 bytes (16 bits) - used for byte access to the frame (bytes[0] is the less significant byte) 
 *  \param  word   (short) : Single word value (16 bits) - used for word access to the frame
 *  @note This structure is a multipupose tool allowing user to access the frame bitwise or bytewise or wordwise 
 *  @note More info can be found here : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf (page 12)
 */
typedef union {
    short       word;
    char        bytes[2];
    T_bits      Bits
}T_packet;

/*********************** AS5047 REGISTERS ***********************/

/**
 *  \struct T_ERRFL_REG
 *  \brief  bitfield that matches ERRFL register of the AS5047. 
 *  \note The bitfield is composed of :
 *  \param  FRERR   (short) : Frame error bit (bit 0) - SPI frame is not valid - true if set
 *  \param  INVCOMM (short) : Invalid command bit (bit 1) - SPI Address is not a valid AS5047P register  - true if set
 *  \param  PARERR  (short) : Parity error bit (bit 2) - Previous frame parity was erroneous - true if set 
 *  @note More info can be found here : https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf (page 15)
 */

/**
 *  \union T_ERRFL_REG
 *  \brief  union of a T_packet to match the frame format of a SPI's AS5047 communication  and the ERRFL Register (address = 0x0001) bit field of the AS5047 objects (objects overlap)
 *  \note This structure is the union of : a 16 bits word (word), a table of 2 bytes (bytes[2]) and a bitfield (Bits), allowing user to access the frame as a short, 2 bytes or frame fields.
 */
typedef union {
    T_packet    packet;
    struct {
        short   FRERR   : 1;
        short   INVCOMM : 1;
        short   PARERR  : 1;
        short           : 13;
    }Bits;
}T_ERRFL_REG;

typedef union {
    T_packet    packet;
    struct {
        short   PROGEN  : 1;
        short           : 1;
        short   OTPREF  : 1;
        short   PROGOTP : 1;
        short           : 2;
        short   PROGVER : 1;
        short           : 9;
    }Bits;
}T_PROG_REG;

typedef union {
    T_packet    packet;
    struct {
        short   AGC     : 8;
        short   LF      : 1;
        short   COF     : 1;
        short   MAGH    : 1;
        short   MAGL    : 1;
        short           : 4;
    }Bits;
}T_DIAAGC_REG;

typedef union {
    T_packet    packet;
    struct {
        short   ZPOSM   : 8;
        short           : 8;
    }Bits;
}T_ZPOSM_REG;

typedef union {
    T_packet    packet;
    struct {
        short   ZPOSL   : 6;
        short   COMP_L  : 1;
        short   COMP_H  : 1;
        short           : 8;
    }Bits;
}T_ZPOSL_REG;

typedef union {
    T_packet    packet;
    struct {
        short   FSET    : 1;
        short           : 1;
        short   DIR     : 1;
        short   UVW_ABI : 1;
        short   DAECDIS : 1;
        short   ABIBIN  : 1;
        short   DATASEL : 1;
        short   PWMON   : 1;
        short           : 8;
    }Bits;
}T_SETTING1_REG;

typedef union {
    T_packet    packet;
    struct {
        short   UVWPP   : 3;
        short   HYS     : 2;
        short   ABIRES  : 3;
        short           : 8;
    }Bits;
}T_SETTING2_REG;

typedef enum {
    dataWrite = 0,
    dataRead = 1
} T_direction;


int ComputeEvenParity (short value);
bool isAddressValid (short address);
bool isDataValid (short value);

T_AS5047_Error isPaquetValid (T_packet packet);
T_AS5047_Error createFrame (T_packet *paquet, short value, T_direction direction = dataWrite);
T_AS5047_Error writeData (short address, short value);
T_AS5047_Error readData (short address, short *value);

protected :

// Objects
SPI* _sensor;

};
#endif