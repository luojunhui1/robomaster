
#include "SerialPort.hpp"

SerialPort::SerialPort(){}

SerialPort::SerialPort(const char* filename, int buadrate)
{
    file_name_ = filename;
    buadrate_ = buadrate;
    success_ = false;
//    serial_mode = NO_INIT;
    fd = open(file_name_, O_RDWR);// Read/Write access to serial port                                           // No terminal will control the process
    last_fd = fd;
    if(fd == -1)
    {
        printf("open_port wait to open %s .\n", file_name_);
//        NOTICE("wait serial " << file_name_,1);
        return;
    }
    else if(fd != -1 )
    {
        fcntl(fd, F_SETFL,0);
//        NOTICE("port is open " << file_name_,1);
        printf("port is open %s.\n", file_name_);
    }
    struct termios port_settings;               // structure to store the port settings in
    if(buadrate_==0)
    {
        cfsetispeed(&port_settings, B115200);       // set baud rates

        cfsetospeed(&port_settings, B115200);
    }
    else if(buadrate_ == 1)
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
    port_settings.c_cc[VMIN]  = 0;            // read doesn't block
    port_settings.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_lflag = ICANON;
    port_settings.c_cc[VMIN] = 10;           // read doesn't block
    port_settings.c_cc[VTIME] = 5;          // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings);             // apply the settings to the port
}
void SerialPort::update_serial_out(float distance_, float last_distance_, float yaw_, float pitch_, bool findstate_)
{
	if (findstate_ = FIND_ARMOR_YES)
		serial_output.curr_distance = distance_;
	else
		serial_output.curr_distance = last_distance_;
	serial_output.findstate = findstate_;
	serial_output.curr_yaw = yaw_;
	serial_output.curr_pitch = pitch_;
	pack();
	return;
}
void SerialPort::send_data()
{
    if(serial_output.size != write(fd, serial_output.res,serial_output.size))
    {
        cout << "!!! send data failure !!!" << fd << endl;
		restart_serial();
		cout << "restart serial!" << endl;
    }
}

void SerialPort::pack()
{
		serial_output.res[0] = (u_char)serial_output.head;
		serial_output.res[serial_output.size - 1] = (u_char)serial_output.end;
		serial_output.res[1] = (u_char)(serial_output.curr_yaw) ;
		//serial_output.res[2] = (uint16_t)((serial_output.curr_yaw) >>8) & 0xff;
		serial_output.res[2] = (u_char)serial_output.curr_pitch;
		//serial_output.res[4] = (uint16_t)((serial_output.curr_pitch) >> 8) & 0xff;
		serial_output.res[3] = (u_char)serial_output.curr_distance;
		//serial_output.res[6] = (uint16_t)(serial_output.curr_distance >> 8) & 0xff;
		return;
}
bool SerialPort::read_data()
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
		restart_serial();
		success_ = false;
		return 0;
	}
	//    printf("buffer1 = %d\t\buffer1 = %d\t buffer1 = %d\tbuffer1 = %d\t\n", read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4]);
	if (read_buffer[0] == serial_input.head && read_buffer[8] == serial_input.end)
	{
		serial_input.curr_yaw = uint8_t(read_buffer[1]);
		serial_input.curr_pitch = uint8_t(read_buffer[2]);
		serial_input.state = uint8_t(read_buffer[3]);
		//        printf("buffer1 = %d\r\n", read_buffer[1]);
		serial_input.anti_top = uint8_t(read_buffer[4]);
		serial_input.enemy_color = uint8_t(read_buffer[5]);
		serial_input.delta_x = uint8_t(read_buffer[6]);
		serial_input.delta_y = uint8_t(read_buffer[7]);
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

void SerialPort::restart_serial(void)
{
	//    cout << "test restart !!" << fd << " " << last_fd << endl;
	close(fd);
	fd = open(file_name_, O_RDWR | O_NOCTTY | O_SYNC);// Read/Write access to serial port
//    cout << serial_mode << endl;
	if (fd == -1 && last_fd != -1)
	{
		printf("open_port wait to open %s .\n", file_name_);
		//        NOTICE("wait serial",1);
		last_fd = fd;
		return;
	}
	else if (fd != -1 && last_fd == -1)
	{
		fcntl(fd, F_SETFL, 0);
		//        NOTICE("port is open",1);
		printf("port is open %s.\n", file_name_);
		last_fd = fd;
	}
	else
	{
		last_fd = fd;
		return;
	}
	struct termios port_settings;               // structure to store the port settings in
	if (buadrate_ == 0)
	{
		cfsetispeed(&port_settings, B115200);       // set baud rates

		cfsetospeed(&port_settings, B115200);
	}
	else if (buadrate_ == 1)
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
