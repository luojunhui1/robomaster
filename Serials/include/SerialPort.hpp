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

//暂定20字节,头1字节,数据18字节,尾1字节
#define    VISION_LENGTH        20
//起始字节,协议固定为0xA5
#define    VISION_SOF         (0xA5)
//end字节,协议固定为0xA5
#define    VISION_TOF         (0xA6)

/**    ----------------------------------------------------------------------------------------------------
FIELD  |  A5  |  CmdID  |  yaw  |  pitch  | distance  |  shoot  |  find  |  none  |  none  |  none |  A6  |
       ----------------------------------------------------------------------------------------------------
BYTE   |   1  |    1    |   4   |    4    |     4     |    1    |    1   |    1   |    1   |   1   |   1  |
       ----------------------------------------------------------------------------------------------------
**/

using namespace std;
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

    explicit Serial(int nSpeed = 115200, char nEvent = 'N', int nBits = 8, int nStop = 1);
    ~Serial();
    void pack(float yaw, float pitch, float dist, uint8_t shoot, uint8_t find, uint8_t CmdID);
    bool InitPort(int nSpeed = 115200, char  nEvent = 'N', int nBits = 8, int nStop = 1);
    bool WriteData();
    bool ReadData(unsigned char* buffer, unsigned int length);
};


#endif //TRANFORM_T_SERIAL_H
