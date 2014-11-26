#ifndef DEFINES_H
#define DEFINES_H

#include <stdint.h>

namespace eri{

typedef uint32_t DeviceId;
typedef uint32_t PacketType;

//Device Class/Type defines
#define	UNKNOWN		0x00
#define	UNSUPPORTED	0xF0
#define SYSTEM		0x01
#define	POSITION	0x02
#define	PROXIMITY	0x03
#define	VISION		0x04
#define COMM		0x05
#define PROXY		0x06

//Device subtypes
enum System_Subtypes{Stats=0x01, Server, NetSession, Client, ClientSession, Testdev};
enum Position_Subtypes{Gpsd=0x01, POS_Compass, Encoder, IMU, PhidgetServoDev, PIDControllerDev};
enum Proximity_Subtypes{Lidar=0x01, Sonar, IR};
enum Vision_Subtypes{OpenCV=0x01, CMUCam};
enum Comm_Subtypes{JAUS=0x01};
enum Proxy_Subtypes{Position=0x01};

//Signal targets
#define AUTO_TARGET	0x00
#define INST_ALL	0xF0
#define TYPE_ALL	0x01
#define	SUBTYPE_ALL	0x02

#define	GET_DEV_TYPE(class, subclass) (unsigned char)((subclass<<4)+class)
#define GET_DEV_ID(type,inst) (unsigned short)((type<<8)+inst)
#define GET_DEV_TYPE_FROM_ID(devId) (uint8_t)(devId>>8)
#define TIMEVAL_TO_MS(t) (uint64_t)((t.tv_sec*1000)+(t.tv_usec/1000))

//Signal types
#define	DEVICE_INIT				0x10
#define DEVICE_READY			0x11
#define	DEVICE_STOPPED			0x12
#define DEVICE_ERROR			0x13
#define DEVICE_CRITICAL_ERROR	0x14
#define DEVICE_DESTROYED		0x15
#define DEVICE_QUERY                    0x16
#define DATA_READY			0x20
#define SIGNAL_DROP			0x50
#define DEVICE_FREE_REQUEST		0x60
#define DATA_REQUEST			0x70
#define HALT_REQUEST			0x80
#define SETTINGS_PUSH                   0x30
#define PID_SET_MODE                    0x4a
#define PID_GET_MODE                    0x4b
#define PID_RESPONSE			0x4c

#define SIG_TYPE_ALL			0xFF

//Network defines
#define	DEFAULT_PORT			6569

//Device Queries
#define	PHIDGET_GET_NUM_OUTS	        0x01
#define PHIDGET_SET_OUT_VAL		0x02

//Phidget Data Types
#define PHIDGET_GET_MOTOR_SPEEDS        0xb0
#define PHIDGET_SET_MOTOR_SPEEDS        0xb1
#define PHIDGET_GET_MOTOR_STOP          0xb2
#define PHIDGET_SET_MOTOR_STOP          0xb3

#define	MOTORS_DRIVE_ALL		0xFA
#define	MOTORS_DRIVE_LEFT		0xFB
#define	MOTORS_DRIVE_RIGHT		0xFC

#define MOTORS_DRIVE_STOP		0xF0

#define DEFAULT_THREAD_TIME		10

#define	LIDAR_SCAN			0x0A

#define CONTROL_HEADING		0xDD

};
#endif
