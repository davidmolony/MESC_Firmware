/*
 * can_ids.h
 *
 *  Created on: 20.02.2023
 *      Author: jensk
 */

#ifndef CAN_IDS_H_
#define CAN_IDS_H_

#define CAN_ID_IQREQ 			0x001
#define CAN_ID_ADC1_2_REQ		0x010


#define CAN_ID_PING				0x29A
#define CAN_ID_TERMINAL			0x29B
#define CAN_ID_CONNECT			0x29C

#define CAN_ID_SPEED 			0x2A0

#define CAN_ID_BUS_VOLT_CURR 	0x2B1

#define CAN_ID_POWER 			0x2B3
#define CAN_ID_TEMP_MOT_MOS1 	0x2B4
#define CAN_ID_TEMP_MOS2_MOS3 	0x2B5
#define CAN_ID_MOTOR_CURRENT	0x2B6
#define CAN_ID_MOTOR_VOLTAGE	0x2B7

#define CAN_ID_STATUS 			0x2C0
#define CAN_ID_FOC_HYPER		0x2C1

#define CAN_ID_SAMPLE			0x300
#define CAN_ID_SAMPLE_NOW		0x301
#define CAN_ID_SAMPLE_SEND		0x302


#define CAN_BROADCAST	0

#define CAN_SAMPLE_FLAG_START	1
#define CAN_SAMPLE_FLAG_END		2






#endif /* CAN_IDS_H_ */
