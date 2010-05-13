/**************************************************************************

    Copyright 2007, 2008 Rainer Jäkel <rainer.jaekel@googlemail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

**************************************************************************/


#ifndef _CONSTANTS
#define _CONSTANTS

#define cbi(REG8, BITNUM) REG8 &= ~(_BV(BITNUM) )
#define sbi(REG8, BITNUM) REG8 |= _BV(BITNUM)

#define ON 1
#define OFF 0
#define _ON 0
#define _OFF 1


//--- Control Table Address ---
//AXS1
#define P_AXS1_DIST_STD_VAL   20
#define P_AXS1_MAX_SOUND_VOL  36

//AX12
//EEPROM AREA
#define P_MODEL_NUMBER_L      0
#define P_MODEL_NUMBER_H      1
#define P_VERSION             2
#define P_ID                  3
#define P_BAUD_RATE           4
#define P_RETURN_DELAY_TIME   5
#define P_CW_ANGLE_LIMIT_L    6
#define P_CW_ANGLE_LIMIT_H    7
#define P_CCW_ANGLE_LIMIT_L   8
#define P_CCW_ANGLE_LIMIT_H   9
#define P_SYSTEM_DATA2        10
#define P_LIMIT_TEMPERATURE   11
#define P_DOWN_LIMIT_VOLTAGE  12
#define P_UP_LIMIT_VOLTAGE    13
#define P_MAX_TORQUE_L        14
#define P_MAX_TORQUE_H        15
#define P_RETURN_LEVEL        16
#define P_ALARM_LED           17
#define P_ALARM_SHUTDOWN      18
#define P_OPERATING_MODE      19
#define P_DOWN_CALIBRATION_L  20
#define P_DOWN_CALIBRATION_H  21
#define P_UP_CALIBRATION_L    22
#define P_UP_CALIBRATION_H    23

#define P_TORQUE_ENABLE         (24)
#define P_LED                   (25)
#define P_CW_COMPLIANCE_MARGIN  (26)
#define P_CCW_COMPLIANCE_MARGIN (27)
#define P_CW_COMPLIANCE_SLOPE   (28)
#define P_CCW_COMPLIANCE_SLOPE  (29)
#define P_GOAL_POSITION_L       (30)
#define P_GOAL_POSITION_H       (31)
#define P_GOAL_SPEED_L          (32)
#define P_GOAL_SPEED_H          (33)
#define P_TORQUE_LIMIT_L        (34)
#define P_TORQUE_LIMIT_H        (35)
#define P_PRESENT_POSITION_L    (36)
#define P_PRESENT_POSITION_H    (37)
#define P_PRESENT_SPEED_L       (38)
#define P_PRESENT_SPEED_H       (39)
#define P_PRESENT_LOAD_L        (40)
#define P_PRESENT_LOAD_H        (41)
#define P_PRESENT_VOLTAGE       (42)
#define P_PRESENT_TEMPERATURE   (43)
#define P_REGISTERED_INSTRUCTION (44)
#define P_PAUSE_TIME            (45)
#define P_MOVING                (46)
#define P_LOCK                  (47)
#define P_PUNCH_L               (48)
#define P_PUNCH_H               (49)

//--- Instruction ---
#define INST_PING           0x01
#define INST_READ           0x02
#define INST_WRITE          0x03
#define INST_REG_WRITE      0x04
#define INST_ACTION         0x05
#define INST_RESET          0x06
#define INST_DIGITAL_RESET  0x07
#define INST_SYSTEM_READ    0x0C
#define INST_SYSTEM_WRITE   0x0D
#define INST_SYNC_WRITE     0x83
#define INST_SYNC_REG_WRITE 0x84

#define CLEAR_BUFFER gbRxBufferReadPointer = gbRxBufferWritePointer
#define DEFAULT_RETURN_PACKET_SIZE 6
#define BROADCASTING_ID 0xfe

//  'OR' these together for error codes
#define STATUS_ERROR_INSTRUCTION    (1<<6)
#define STATUS_ERROR_OVERLOAD       (1<<5)
#define STATUS_ERROR_CHECKSUM       (1<<4)
#define STATUS_ERROR_RANGE          (1<<3)
#define STATUS_ERROR_OVERHEATING    (1<<2)
#define STATUS_ERROR_ANGLE_LIMIT    (1<<1)
#define STATUS_ERROR_INPUT_VOLTAGE  (1<<0)

//Hardware Dependent Item
#define DEFAULT_BAUD_RATE 34   //57600bps at 16MHz, 34=57.6k, 0 = 2MBit

////// For CM-5
#define RS485_TXD PORTE &= ~_BV(PE3), PORTE |= _BV(PE2)  //_485_DIRECTION = 1
#define RS485_RXD PORTE &= ~_BV(PE2), PORTE |= _BV(PE3)  //PORT_485_DIRECTION = 0

/*
   ////// For CM-2
 #define RS485_TXD PORTE |= _BV(PE2); //_485_DIRECTION = 1
 #define RS485_RXD PORTE &= ~_BV(PE2);//PORT_485_DIRECTION = 0
 */
//#define TXD0_FINISH  UCSR0A,6  //This bit is for checking TxD Buffer in CPU is empty or not.
//#define TXD1_FINISH	 UCSR1A,6

#define SET_TxD0_FINISH   sbi(UCSR0A, 6)
#define RESET_TXD0_FINISH cbi(UCSR0A, 6)
#define CHECK_TXD0_FINISH bit_is_set(UCSR0A, 6)
#define SET_TxD1_FINISH  sbi(UCSR1A, 6)
#define RESET_TXD1_FINISH cbi(UCSR1A, 6)
#define CHECK_TXD1_FINISH bit_is_set(UCSR1A, 6)

#define RX_INTERRUPT 0x01
#define TX_INTERRUPT 0x02
#define OVERFLOW_INTERRUPT 0x01
#define SERIAL_PORT0 0
#define SERIAL_PORT1 1
#define BIT_RS485_DIRECTION0  0x08  //Port E
#define BIT_RS485_DIRECTION1  0x04  //Port E

#define BIT_ZIGBEE_RESET               PD4  //out : default 1 //PORTD
#define BIT_ENABLE_RXD_LINK_PC         PD5  //out : default 1
#define BIT_ENABLE_RXD_LINK_ZIGBEE     PD6  //out : default 0
#define BIT_LINK_PLUGIN                PD7  //in, no pull up

////// For the IMU (taking the lead from HUV's control block)
#define P_IMUHUV_LED               (25)
#define P_IMUHUV_SIDEWAYS_ACCEL_L  (26)
#define P_IMUHUV_SIDEWAYS_ACCEL_H  (27)
#define P_IMUHUV_FOREWARDS_ACCEL_L (28)
#define P_IMUHUV_FOREWARDS_ACCEL_H (29)
#define P_IMUHUV_VERTICAL_ACCEL_L  (30)
#define P_IMUHUV_VERTICAL_ACCEL_H  (31)
#define P_IMUHUV_PITCH_RATE_L      (32)
#define P_IMUHUV_PITCH_RATE_H      (33)
#define P_IMUHUV_ROLL_RATE_L       (34)
#define P_IMUHUV_ROLL_RATE_H       (35)
#define P_IMUHUV_YAW_RATE_L        (36)
#define P_IMUHUV_YAW_RATE_H        (37)

#endif
