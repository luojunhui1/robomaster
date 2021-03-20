#include "SerialPort.hpp"

bool wait_uart = false;

string get_uart_dev_name() {
    FILE *ls = popen("ls /dev/ttyTHS* --color=never", "r");
    char name[20] = {0};
    fscanf(ls, "%s", name);
    pclose(ls);
    return name;
}

Serial::Serial(int nSpeed, char nEvent, int nBits, int nStop) :
        nSpeed(nSpeed), nEvent(nEvent), nBits(nBits), nStop(nStop) {
    if (wait_uart) {
        cout<<("Wait for serial be ready!")<<endl;
        InitPort(nSpeed, nEvent, nBits, nStop);
        cout<<("Port set successfully!")<<endl;
    } else {
        if (InitPort(nSpeed, nEvent, nBits, nStop)) {
            cout<<("Port set successfully!")<<endl;
        } else {
            cout<<("Port set fail!")<<endl;
        }
    }
}

Serial::~Serial() {
    close(fd);
    fd = -1;
}

bool Serial::InitPort(int nSpeed_, char nEvent_, int nBits_, int nStop_) {
    string name = PC2STM32;
    if (name.empty()) {
        return false;
    }
    if ((fd = open(name.data(), O_RDWR|O_APPEND|O_SYNC)) < 0) {
        cout<<"fd failed!"<<endl;
        return false;
    }
    return set_opt(fd, nSpeed_,nEvent_, nBits_, nStop_) >= 0;
}

void Serial::pack(float yaw, float pitch, float dist, uint8_t shoot, uint8_t find, uint8_t CmdID, long long timeStamp)
{
    unsigned char *p;
    buff[0] = VISION_SOF;
//    buff[1] = CmdID;
    memcpy(buff + 1, &CmdID, 1);
    memcpy(buff + 2, &yaw, 4);
    memcpy(buff + 6, &pitch, 4);
    memcpy(buff + 10, &dist, 4);
    memcpy(buff + 14, &shoot, 4);
    memcpy(buff + 15, &find, 4);
    memcpy(buff + 16, &timeStamp, 8);

//    p = (unsigned char*)&yaw;
//    buff[2] = static_cast<unsigned char>(*p);
//    buff[3] = static_cast<char>(*(p+1));
//    buff[4] = static_cast<char>(*(p+2));
//    buff[5] = static_cast<char>(*(p+3));
//
//    p = (unsigned char*)&pitch;
//    buff[6] = static_cast<unsigned char>(*p);
//    buff[7] = static_cast<char>(*(p+1));
//    buff[8] = static_cast<char>(*(p+2));
//    buff[9] = static_cast<char>(*(p+3));
//
//    p = (unsigned char*)&dist;
//    buff[10] = static_cast<unsigned char>(*p);
//    buff[11] = static_cast<char>(*(p+1));
//    buff[12] = static_cast<char>(*(p+2));
//    buff[13] = static_cast<char>(*(p+3));
//
//    buff[14] = static_cast<char>(shoot);
//    buff[15] = static_cast<unsigned char>(find);
//
//    p = (unsigned char*)&timeStamp;
//    buff[16] = static_cast<unsigned char>(*p);
//    buff[17] = static_cast<char>(*(p + 1));
//    buff[18] = static_cast<char>(*(p + 2));
//    buff[19] = static_cast<char>(*(p + 3));
//    buff[20] = static_cast<char>(*(p + 4));
//    buff[21] = static_cast<char>(*(p + 5));
//    buff[22] = static_cast<char>(*(p + 6));
//    buff[23] = static_cast<char>(*(p + 7));
    buff[24] = static_cast<char>(VISION_TOF);
}

bool Serial::WriteData() {
    int cnt = 0, curr = 0;
    if (fd <= 0){
        if(wait_uart){
            InitPort(nSpeed, nEvent, nBits, nStop);
        }
        return false;
    }
    //cout<<"Write Begin to USB!!!!!!!!!!!!!!!!!"<<endl;
    curr = write(fd, buff, VISION_LENGTH);
    //cout<<"Write Over to USB!!!!!!!!!!!!!!!!!"<<endl;
    if (curr < 0) {
        cout<<("Write Serial offline!")<<endl;
        close(fd);
        if (wait_uart) {
            InitPort(nSpeed, nEvent, nBits, nStop);
        }
        return false;
    }
    return true;
}

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
            cout << e.what() << endl;
            return false;
        }

        if (onceReadCount < 1)
        {
            return false;
        }
        readCount += onceReadCount;
    }

//	for(int i = 0; i < VISION_LENGTH; i++)
//	{
//		printf("%x\t",buffRead[i]);
//	}
//	printf("\n");

    if (buffRead[0] != VISION_SOF || buffRead[VISION_LENGTH - 1] != VISION_TOF)
    {
        return false;
    }
    else
    {
        memcpy(&buffer_.yawAngle,buffRead + 1,4);
        memcpy(&buffer_.pitchAngle,buffRead + 5,4);
        memcpy(&buffer_.yawSpeed,buffRead + 9,4);
        memcpy(&buffer_.pitchSpeed,buffRead + 13,4);
        memcpy(&buffer_.targetMode,buffRead + 17,1);
        return true;
    }

}

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
