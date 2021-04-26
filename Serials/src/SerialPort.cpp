#include "SerialPort.hpp"

bool wait_uart = false;

/**
 * @brief list all the available uart resources
 * @return uart's name that user appointed
 */
string get_uart_dev_name() {
    FILE *ls = popen("ls /dev/ttyTHS* --color=never", "r");
    char name[20] = {0};
    fscanf(ls, "%s", name);
    pclose(ls);
    return name;
}

/**
 * @brief constructor of Serial class
 * @param nSpeed baud rate
 * @param nEvent verification mode, odd parity, even parity or none
 * @param nBits the length of data
 * @param nStop length of stop bits
 * @return none
 */
Serial::Serial(int nSpeed, char nEvent, int nBits, int nStop) :
        nSpeed(nSpeed), nEvent(nEvent), nBits(nBits), nStop(nStop) {
    if (wait_uart) {
        LOGA("Wait for serial be ready!");
        //cout<<("Wait for serial be ready!")<<endl;
        InitPort(nSpeed, nEvent, nBits, nStop);
        LOGA("Port set successfully!");
        //cout<<("Port set successfully!")<<endl;
    } else {
        if (InitPort(nSpeed, nEvent, nBits, nStop)) {
            LOGA("Port set successfully!");
        } else {
            LOGE("Port set fail!");
            //cout<<("Port set fail!")<<endl;
        }
    }
}

/**
 * @brief destructor of Serial class
 * @return none
 */
Serial::~Serial() {
    close(fd);
    fd = -1;
}

/**
 * @brief initialize port
 * @param nSpeed_ baud rate
 * @param nEvent_ verification mode, odd parity, even parity or none
 * @param nBits_ the length of data
 * @param nStop_ length of stop bits
 * @return none
 */
bool Serial::InitPort(int nSpeed_, char nEvent_, int nBits_, int nStop_) {
    string name = PC2STM32;
    if (name.empty()) {
        return false;
    }
    if ((fd = open(name.data(), O_RDWR|O_APPEND|O_SYNC)) < 0) {
        LOGE("fd failed!");
        //cout<<"fd failed!"<<endl;
        return false;
    }
    return set_opt(fd, nSpeed_,nEvent_, nBits_, nStop_) >= 0;
}

/**
 * @brief package the data needed by lower computer
 * @param yaw yaw angle
 * @param pitch pitch angle
 * @param dist distance of target armor
 * @param shoot shoot or not
 * @param find found target armor or not
 * @param CmdID command id
 * @param timeStamp time stamp
 * @details CmdID and timeStamo not really used in current code
 * @return none
 */
void Serial::pack(float yaw, float pitch, float dist, uint8_t shoot, uint8_t find, uint8_t CmdID, long long timeStamp)
{
    unsigned char *p;
    buff[0] = VISION_SOF;
    memcpy(buff + 1, &CmdID, 1);
    memcpy(buff + 2, &yaw, 4);
    memcpy(buff + 6, &pitch, 4);
    memcpy(buff + 10, &dist, 4);
    memcpy(buff + 14, &shoot, 4);
    memcpy(buff + 15, &find, 4);
    memcpy(buff + 16, &timeStamp, 8);
    buff[24] = static_cast<char>(VISION_TOF);
}

/**
 * @brief write data to port
 * @return always should be true
 */
bool Serial::WriteData() {
    int cnt = 0, curr = 0;
    if (fd <= 0){
        if(wait_uart){
            InitPort(nSpeed, nEvent, nBits, nStop);
        }
        return false;
    }
    //cout<<"Write Begin to USB!!!!!!!!!!!!!!!!!"<<endl;

    tcflush(fd, TCOFLUSH);
    curr = write(fd, buff, VISION_LENGTH);

    //cout<<"Write Over to USB!!!!!!!!!!!!!!!!!"<<endl;
    if (curr < 0) {
        LOGW("Write Serial offline!");
        //cout<<("Write Serial offline!")<<endl;
        close(fd);
        if (wait_uart) {
            InitPort(nSpeed, nEvent, nBits, nStop);
        }
        return false;
    }

    return true;
}

/**
 * @brief read data sent by lower computer
 * @param buffer_ instance should be updated by data sent from lower computer
 * @return on finding the right data in a limited length of received data, return true, if not, return false
 */
bool Serial::ReadData(struct ReceiveData &buffer_) {
    memset(buffRead,0,VISION_LENGTH);
    maxReadTime = VISION_LENGTH;
    static int onceReadCount = 0;

    tcflush(fd, TCIFLUSH);
    
    while(maxReadTime--)
    {
        read(fd, &buffRead[0], 1);
        if(buffRead[0] == 0xA5)break;
    }

    if(maxReadTime == 0)return false;

    readCount = 1;
    while (readCount < VISION_LENGTH - 1)
    {
        try
        {
            onceReadCount = read(fd, (buffRead + readCount), VISION_LENGTH - readCount);
        }
        catch(exception e)
        {
            LOGE("Data Read Error!");
            //cout << e.what() << endl;
            return false;
        }

        if (onceReadCount < 1)
        {
            return false;
        }
        readCount += onceReadCount;
    }

    if (buffRead[0] != VISION_SOF || buffRead[VISION_LENGTH - 1] != VISION_TOF)
    {
        return false;
    }
    else
    {
        memcpy(&buffer_.yawAngle,buffRead + 2,4);
        memcpy(&buffer_.pitchAngle,buffRead + 6,4);
        memcpy(&buffer_.yawSpeed,buffRead + 10,4);
        memcpy(&buffer_.pitchSpeed,buffRead + 14,4);
        memcpy(&buffer_.targetMode,buffRead + 18,1);
        memcpy(&buffer_.targetColor,buffRead + 19,1);
        memcpy(&buffer_.direction,buffRead + 20,1);
        return true;
    }

}

/**
 * @brief set port
 * @param fd file or port descriptor
 * @param nSpeed baud rate
 * @param nEvent verification mode, odd parity, even parity or none
 * @param nBits length of data
 * @param nStop length of stop bits
 * @return
 */
int Serial::set_opt(int fd, int nSpeed, char nEvent, int nBits, int nStop) {
    termios newtio{}, oldtio{};

    if (tcgetattr(fd, &oldtio) != 0) {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (nBits) {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
        default:
            break;
    }

    switch (nEvent) {
        case 'O':  //奇校验
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':  //偶校验
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':  //无校验
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            break;
    }

    switch (nSpeed) {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }

    if (nStop == 1) {
        newtio.c_cflag &= ~CSTOPB;
    } else if (nStop == 2) {
        newtio.c_cflag |= CSTOPB;
    }
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    newtio.c_oflag &= ~OPOST;
    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN] = 1;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");

    return 0;
}
