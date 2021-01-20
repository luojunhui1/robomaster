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

void Serial::pack(float yaw, float pitch, float dist, uint8_t shoot, uint8_t find, uint8_t CmdID)
{
    unsigned char *p;
    buff[0] = VISION_SOF;
    buff[1] = CmdID;
    p = (unsigned char*)&yaw;
    buff[2] = static_cast<unsigned char>(*p);
    buff[3] = static_cast<char>(*(p+1));
    buff[4] = static_cast<char>(*(p+2));
    buff[5] = static_cast<char>(*(p+3));
    p = (unsigned char*)&pitch;
    buff[6] = static_cast<unsigned char>(*p);
    buff[7] = static_cast<char>(*(p+1));
    buff[8] = static_cast<char>(*(p+2));
    buff[9] = static_cast<char>(*(p+3));
    p = (unsigned char*)&dist;
    buff[10] = static_cast<unsigned char>(*p);
    buff[11] = static_cast<char>(*(p+1));
    buff[12] = static_cast<char>(*(p+2));
    buff[13] = static_cast<char>(*(p+3));

    buff[14] = static_cast<char>(shoot);
    buff[15] = static_cast<unsigned char>(find);
    buff[16] = static_cast<char>(0x0000);
    buff[17] = static_cast<char>(0x0000);
    buff[18] = static_cast<char>(0x0000);
    buff[19] = static_cast<char>(VISION_TOF);
}

bool Serial::WriteData() {
    int cnt = 0, curr = 0;
    if (fd <= 0){
        if(wait_uart){
            InitPort(nSpeed, nEvent, nBits, nStop);
        }
        return false;
    }
    curr = write(fd, buff, VISION_LENGTH);
    if (curr < 0) {
        cout<<("Serial offline!")<<endl;
        close(fd);
        if (wait_uart) {
            InitPort(nSpeed, nEvent, nBits, nStop);
        }
        return false;
    }
    return true;
}

bool Serial::ReadData(unsigned char *buffer, unsigned int length) {
    int cnt = 0, curr = 0;
    while ((curr = read(fd, buffer + cnt, length - cnt)) > 0 && (cnt += curr) < length);
    if (curr < 0) {
        cout<<("Serial offline!")<<endl;
        close(fd);
        if (wait_uart) {
            InitPort(nSpeed, nEvent, nBits, nStop);
        }
        return false;
    }
    return true;
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

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");

    return 0;
}
static void sleep_ms(unsigned int secs)
{

    struct timeval tval;

    tval.tv_sec=secs/1000;

    tval.tv_usec=(secs*1000)%1000000;

    select(0,NULL,NULL,NULL,&tval);

}