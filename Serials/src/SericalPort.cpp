
#include "SerialPort.hpp"

SerialPort::SerialPort(){}

SerialPort::SerialPort(const char* filename, int buadrate)
{
    fileName = filename;
    buadrate = buadrate;
    success_ = false;
//    serial_mode = NO_INIT;
    fd = open(fileName, O_RDWR);// Read/Write access to serial port                                           // No terminal will control the process
    lastFd = fd;
    if(fd == -1)
    {
        printf("open_port wait to open %s .\n", fileName);
//        NOTICE("wait serial " << fileName,1);
        return;
    }
    else if(fd != -1 )
    {
        fcntl(fd, F_SETFL,0);
//        NOTICE("port is open " << fileName,1);
        printf("port is open %s.\n", fileName);
    }
    struct termios portSettings{};               // structure to store the port settings in
    if(buadrate == 0)
    {
        cfsetispeed(&portSettings, B115200);       // set baud rates

        cfsetospeed(&portSettings, B115200);
    }
    else if(buadrate == 1)
    {
        cfsetispeed(&portSettings, B921600);       // set baud rates
        cfsetospeed(&portSettings, B921600);
    }
    portSettings.c_cflag = (portSettings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    portSettings.c_iflag &= ~IGNBRK;         // disable break processing
    portSettings.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    portSettings.c_oflag = 0;                // no remapping, no delays
    portSettings.c_cc[VMIN]  = 0;            // read doesn't block
    portSettings.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    portSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    portSettings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    portSettings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    portSettings.c_cflag |= 0;
    portSettings.c_cflag &= ~CSTOPB;
    portSettings.c_cflag &= ~CRTSCTS;
    portSettings.c_lflag = ICANON;
    portSettings.c_cc[VMIN] = 10;           // read doesn't block
    portSettings.c_cc[VTIME] = 5;          // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &portSettings);             // apply the settings to the port
}
void SerialPort::UpdateSerialOut(float distance_,float yaw_, float pitch_, int findstate_)
{
    serialOutput.curDistance = distance_;
    serialOutput.findState = findstate_;
    serialOutput.curYaw = yaw_;
    serialOutput.curPitch = pitch_;
	pack();
}
void SerialPort::SendData()
{
    if(serialOutput.size != write(fd, serialOutput.res, serialOutput.size))
    {
        cout << "!!! send data failure !!!" << fd << endl;
        RestartSerial();
		cout << "restart serial!" << endl;
    }
}

void SerialPort::pack()
{
    serialOutput.res[0] = (u_char)serialOutput.head;
    serialOutput.res[serialOutput.size - 1] = (u_char)serialOutput.end;
    serialOutput.res[1] = (u_char)(serialOutput.curYaw) ;
    //serialOutput.res[2] = (uint16_t)((serialOutput.curYaw) >>8) & 0xff;
    serialOutput.res[2] = (u_char)serialOutput.curPitch;
    //serialOutput.res[4] = (uint16_t)((serialOutput.curPitch) >> 8) & 0xff;
    serialOutput.res[3] = (u_char)serialOutput.curDistance;
    //serialOutput.res[6] = (uint16_t)(serialOutput.curDistance >> 8) & 0xff;
}
bool SerialPort::ReadData()
{
	tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */
	unsigned char read_buffer[9];   /* Buffer to store the data received              */
	int  bytes_read = 0;    /* Number of bytes read by the read() system call */

	bytes_read = read(fd, &read_buffer, 9); /* Read the data                   */
//    cout << "bytes_read: " << bytes_read;
//    for (int i = 0; i < bytes_read; i++) {
//        cout << "buf " << i << ": " << read_buffer[i];
//    }
//    cout << endl;
	if (bytes_read == -1 || bytes_read == 0)
	{
		//        cout << "can not read!" << endl;
		//        NOTICE("can not read!",3);
        RestartSerial();
		success_ = false;
		return 0;
	}
	//    printf("buffer1 = %d\t\buffer1 = %d\t buffer1 = %d\tbuffer1 = %d\t\n", read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4]);
	if (read_buffer[0] == serialInput.head && read_buffer[8] == serialInput.end)
	{
        serialInput.curr_yaw = uint8_t(read_buffer[1]);
        serialInput.curr_pitch = uint8_t(read_buffer[2]);
        serialInput.state = uint8_t(read_buffer[3]);
		//        printf("buffer1 = %d\r\n", read_buffer[1]);
		serialInput.anti_top = uint8_t(read_buffer[4]);
        serialInput.enemy_color = uint8_t(read_buffer[5]);
        serialInput.delta_x = uint8_t(read_buffer[6]);
        serialInput.delta_y = uint8_t(read_buffer[7]);
		//        cout << "x: " << buff_offset_x << " y:" << buff_offset_y << endl;
				//        gimbal_data = float(short((read_buffer[4]<<8) | read_buffer[3]))/100.0f;
		//        cout << gimbal_data<< endl;
		success_ = true;
		return 1;
	}
	//    cout << "count "<< cccc << endl;
	success_ = false;
	return 0;
}

void SerialPort::RestartSerial(void)
{
	//    cout << "test restart !!" << fd << " " << lastFd << endl;
	close(fd);
	fd = open(fileName, O_RDWR | O_NOCTTY | O_SYNC);// Read/Write access to serial port
//    cout << serial_mode << endl;
	if (fd == -1 && lastFd != -1)
	{
		printf("open_port wait to open %s .\n", fileName);
		//        NOTICE("wait serial",1);
		lastFd = fd;
		return;
	}
	else if (fd != -1 && lastFd == -1)
	{
		fcntl(fd, F_SETFL, 0);
		//        NOTICE("port is open",1);
		printf("port is open %s.\n", fileName);
        lastFd = fd;
	}
	else
	{
        lastFd = fd;
		return;
	}
	struct termios port_settings;               // structure to store the port settings in
	if (buadrate == 0)
	{
		cfsetispeed(&port_settings, B115200);       // set baud rates

		cfsetospeed(&port_settings, B115200);
	}
	else if (buadrate == 1)
	{
		cfsetispeed(&port_settings, B921600);       // set baud rates
		cfsetospeed(&port_settings, B921600);
	}
	port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	port_settings.c_iflag &= ~IGNBRK;         // disable break processing
	port_settings.c_lflag = 0;                // no signaling chars, no echo,
	// no canonical processing
	port_settings.c_oflag = 0;                // no remapping, no delays
	port_settings.c_cc[VMIN] = 0;            // read doesn't block
	port_settings.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	port_settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	// enable reading
	port_settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	port_settings.c_cflag |= 0;
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CRTSCTS;
	port_settings.c_iflag = ICANON;
	port_settings.c_cc[VMIN] = 10;           // read doesn't block
	port_settings.c_cc[VTIME] = 5;          // 0.5 seconds read timeout

	tcsetattr(fd, TCSANOW, &port_settings);             // apply the settings to the port

}
