//
// Created by luojunhui on 7/22/20.
//

#ifndef TRANFORM_T_SERIAL_H
#define TRANFORM_T_SERIAL_H
#include <iostream>
#include <unistd.h>     // UNIX Standard Definitions
#include <fcntl.h>      // File Control Definitions
#include <cerrno>      // ERROR Number Definitions
#include <termios.h>
#include <cstring>
#include <cstdio>

#define PC2STM32 "/dev/ttyUSB0"//串口位置


/*--------------------------------暂定协议-------------------------------------*/

#define    VISION_LENGTH        20     		 //暂定22字节,头3字节,数据17字节,尾2字节

//起始字节,协议固定为0xA5
#define    VISION_SOF         (0xA5)

//长度根据协议定义,数据段长度为n需要根据帧头第二字节来获取
#define    VISION_LEN_HEADER    2         //帧头长
#define    VISION_LEN_DATA      17        //数据段长度,可自定义
#define    VISION_LEN_TAIL      1	      //帧尾CRC8
#define    VISION_LEN_PACKED    22        //数据包长度,可自定义

#define    VISION_OFF         		(0x00)
#define    VISION_RED           	(0x01)
#define    VISION_BLUE          	(0x02)
#define    VISION_RBUFF_ANTI   	 	(0x03)//红逆大符
#define    VISION_BBUFF_ANTI   		(0x04)//蓝逆大符
#define    VISION_RBUFF_CLOCKWISE   (0x05)//红顺大符
#define    VISION_BBUFF_CLOCKWISE   (0x06)//蓝顺大符
#define    VISION_RBUFF_STAND   	(0x07)//红小符
#define    VISION_BBUFF_STAND   	(0x08)//蓝小符
#define    CRC8                     0XA6
/* 	STM32 -> PC

	CmdID   0x00   关闭视觉
	CmdID   0x01   识别红色装甲
	CmdID   0x02   识别蓝色装甲
	CmdID   0x03   红符
	CmdID   0x04   蓝符
*/

/* 	PC -> STM32

	CmdID   0x00   关闭视觉
	CmdID   0x01   识别红色装甲
	CmdID   0x02   识别蓝色装甲
	CmdID   0x03   小符
	CmdID   0x04   大符
*/

//可利用收和发的指令码进行比较,当收和发的指令码相同时,可判定为数据可用

//帧头加CRC8校验,保证发送的指令是正确的

//PC收发与STM32收发成镜像关系,以下结构体适用于STM32,PC需稍作修改

//typedef  struct
//{
//    /* 头 */
//    uint8_t   SOF;			//帧头起始位,暂定0xA5
//    uint8_t   CmdID;		//指令
//    uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
//
//}extVisionSendHeader_t;


using namespace std;

//typedef struct serial_transmit_data_out//输入到下位机的数据
//{
//    /* 头 */
//    uint8_t   SOF;			//帧头起始位,暂定0xA5
//    uint8_t   CmdID;		//指令
//
//    /* 数据 */
//    float     pitch_angle;
//    float     yaw_angle;
//    float     distance;			//距离
//    uint8_t   centre_lock;		//是否瞄准到了中间  0没有  1瞄准到了
//    uint8_t	  identify_target;	//视野内是否有目标/是否识别到了目标   0否  1是
//    uint8_t   identify_buff;	//打符时是否识别到了目标，1是，2识别到切换了装甲，0没识别到
//
//    uint8_t	  blank_b;			//预留
//    uint8_t	  auto_too_close;   //目标距离太近,视觉发1，否则发0
//
//    /* 尾 */
//    uint8_t  CRC8;
//}SerialOut;
//typedef struct serial_transmit_data_in//上位机接收的数据
//{
//    //	/* 头 */
//    uint8_t   SOF;			//帧头起始位,暂定0xA5
//    uint8_t   CmdID;		//指令
//    /* 数据 */
//    float     pitch_angle;
//    float     yaw_angle;
//    float     distance;			//距离
//    uint8_t   lock_sentry;		//是否在抬头识别哨兵
//    uint8_t   base;				//吊射
//
//    uint8_t   blank_a;		//预留
//    uint8_t	  blank_b;
//    uint8_t	  blank_c;
//
//    /* 尾 */
//    uint8_t  CRC8;
//}SerialIn;
/**
 * @brief SerialPort
 * @param filename 串口名字
 * @param buadrate 串口波特率,用于stm32通信是0-B115200,用于stm32通信是1-B921600
 */
class Serial
{
private:
    int fd;
    int nSpeed;
    char nEvent;
    int nBits;
    int nStop;
    uint8_t buff[VISION_LENGTH];
    static int set_opt(int fd, int nSpeed, char nEvent, int nBits, int nStop);
public:

    Serial(int nSpeed = 9600, char nEvent = 'N', int nBits = 8, int nStop = 1);
    ~Serial();
    void pack(float pitch, float yaw, float dist, uint8_t centerLock, uint8_t targetMode,uint8_t buffMode);
    bool InitPort(int nSpeed = 115200, char  nEvent = 'N', int nBits = 8, int nStop = 1);
    bool WriteData();
    bool ReadData(unsigned char* buffer, unsigned int length);
};


#endif //TRANFORM_T_SERIAL_H
