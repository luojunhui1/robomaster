/*
 * @Descriptions: 
 * @Author: 罗俊辉
 * @Blog: https://luojunhui1.github.io/
 * @Date: 2020-01-09 16:30:28
 * @LastEditors  : 罗俊辉
 * @LastEditTime : 2020-01-09 16:41:35
 */
#pragma once
#include <iostream>
#include <unistd.h>     // UNIX Standard Definitions
#include <fcntl.h>      // File Control Definitions
#include <cerrno>      // ERROR Number Definitions
#include <termios.h>    // POSIX Terminal Control Definitions
#include "mydefine.h"
using namespace std;
#define PC2STM32 "/dev/ttyTHS1"//串口位置

typedef struct serial_transmit_data_out//输入到下位机的数据
{
	uint8_t curYaw = 0;
	uint8_t curPitch = 0;
	uint8_t curDistance = 100;
	bool findState = 0;
	uint8_t size = 5;
	unsigned char res[20]{};
	unsigned char head = 0xaa;
	unsigned char end = 0xbb;
}SerialOut;
typedef struct serial_transmit_data_in//上位机接收的数据
{
	float curr_yaw = 0;      // 当前云台yaw角度
	float curr_pitch = 0;    // 当前云台pitch角
	uint8_t state = AUTO_SHOOT_STATE;       // 当前状态，自瞄-大符-小符
	uint8_t anti_top = 0;    // 是否为反陀螺模式
	uint8_t enemy_color{}; // 敌方颜色
	uint8_t delta_x{};         // 能量机关x轴补偿量
	uint8_t delta_y{};         // 能量机关y轴补偿量
	unsigned char head = 0xaa;
	unsigned char end = 0xbb;
}SerialIn;

class SerialPort
{
public:
    SerialPort();
    /**
     * @brief SerialPort
     * @param filename 串口名字
     * @param buadrate 串口波特率,用于stm32通信是0-B115200,用于stm32通信是1-B921600
     */
    SerialPort(const char* filename, int buadrate);
	/*尝试重连*/
	void RestartSerial(void);
    void SendData();
	bool ReadData();
	void UpdateSerialOut(float distance_, float yaw_, float pitch_, int findstate);
	void pack();
    int fd{};
    int lastFd{};
    bool success_{};

private:
	SerialIn serialInput;
	SerialOut serialOutput;
    const char* fileName{};
    int buadrate{};
    float lastBulletSpeed{};
};

