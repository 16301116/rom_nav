


#include "serial/Serial_Data.h"
#include <ctype.h>
#include <sys/ioctl.h>
#include <sys/termios.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */


//Analy_Sensor_Data analy_dvl_data;


Serial_Data::Serial_Data()
{
	
	
}

void Serial_Data::setNewOptions( int m_fd , int baudrate, int databits, string parity, string stop, bool softwareHandshake, bool hardwareHandshake)
{
	   struct termios newtio;
	  //memset(&newtio, 0, sizeof(newtio));
	   if (tcgetattr(m_fd, &newtio)!=0)
	   {
	      std::cerr<<"tcgetattr() 3 failed"<<std::endl;
	   }

	   speed_t _baud=0;

	   switch (baudrate)
	   {
	#ifdef B0
	   case      0: _baud=B0;     break;
	#endif
	   
	#ifdef B50
	   case     50: _baud=B50;    break;
	#endif
	#ifdef B75
	   case     75: _baud=B75;    break;
	#endif
	#ifdef B110
	   case    110: _baud=B110;   break;
	#endif
	#ifdef B134
	   case    134: _baud=B134;   break;
	#endif
	#ifdef B150
	   case    150: _baud=B150;   break;
	#endif
	#ifdef B200
	   case    200: _baud=B200;   break;
	#endif
	#ifdef B300
	   case    300: _baud=B300;   break;
	#endif
	#ifdef B600
	   case    600: _baud=B600;   break;
	#endif
	#ifdef B1200
	   case   1200: _baud=B1200;  break;
	#endif
	#ifdef B1800
	   case   1800: _baud=B1800;  break;
	#endif
	#ifdef B2400
	   case   2400: _baud=B2400;  break;
	#endif
	#ifdef B4800
	   case   4800: _baud=B4800;  break;
	#endif
	#ifdef B7200
	   case   7200: _baud=B7200;  break;
	#endif
	#ifdef B9600
	   case   9600: _baud=B9600;  break;
	#endif
	#ifdef B14400
	   case  14400: _baud=B14400; break;
	#endif
	#ifdef B19200
	   case  19200: _baud=B19200; break;
	#endif
	#ifdef B28800
	   case  28800: _baud=B28800; break;
	#endif
	#ifdef B38400
	   case  38400: _baud=B38400; break;
	#endif
	#ifdef B57600
	   case  57600: _baud=B57600; break;
	#endif
	#ifdef B76800
	   case  76800: _baud=B76800; break;
	#endif
	#ifdef B115200
	   case 115200: _baud=B115200; break;
	#endif
	#ifdef B128000
	   case 128000: _baud=B128000; break;
	#endif
	#ifdef B230400
	   case 230400: _baud=B230400; break;
	#endif
	#ifdef B460800
	   case 460800: _baud=B460800; break;
	#endif
	#ifdef B576000
	   case 576000: _baud=B576000; break;
	#endif
	#ifdef B600000
	   case 600000: _baud=B600000; break;
	#endif
	#ifdef B921600
	   case 921600: _baud=B921600; break;
	#endif
	   default:
	      break;
	   }

	   cfsetospeed(&newtio, (speed_t)_baud);
	   cfsetispeed(&newtio, (speed_t)_baud);

	   /* We generate mark and space parity ourself. */
	   if (databits == 7 && (parity=="Mark" || parity == "Space"))
	   {
	      databits = 8;
	   }
	   switch (databits)
	   {
	   case 5:
	      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS5;
	      break;
	   case 6:
	      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS6;
	      break;
	   case 7:
	      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS7;
	      break;
	   case 8:
	   default:
	      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8;
	      break;
	   }
	   newtio.c_cflag |= CLOCAL | CREAD;

	   //parity
	   newtio.c_cflag &= ~(PARENB | PARODD);
	   if (parity == "Even")
	   {
	      newtio.c_cflag |= PARENB;
	   }
	   else if (parity== "Odd")
	   {
	      newtio.c_cflag |= (PARENB | PARODD);
	   }

	   //hardware handshake
	/*   if (hardwareHandshake)
	      newtio.c_cflag |= CRTSCTS;
	   else
	      newtio.c_cflag &= ~CRTSCTS;*/
	   newtio.c_cflag &= ~CRTSCTS;

	   //stopbits
	   if (stop=="2")
	   {
	      newtio.c_cflag |= CSTOPB;
	   }
	   else
	   {
	      newtio.c_cflag &= ~CSTOPB;
	   }

	//   newtio.c_iflag=IGNPAR | IGNBRK;
	   newtio.c_iflag=IGNBRK;
	//   newtio.c_iflag=IGNPAR;

	   //software handshake
	   if (softwareHandshake)
	   {
	      newtio.c_iflag |= IXON | IXOFF;
	   }
	   else
	   {
	      newtio.c_iflag &= ~(IXON|IXOFF|IXANY);
	   }

	   newtio.c_lflag=0;
	   newtio.c_oflag=0;

	   newtio.c_cc[VTIME]=1;
	   newtio.c_cc[VMIN]=60;

	//   tcflush(m_fd, TCIFLUSH);
	   if (tcsetattr(m_fd, TCSANOW, &newtio)!=0)
	   {
	      std::cerr<<"tcsetattr() 1 failed"<<std::endl;
	   }

	   int mcs=0;
	   ioctl(m_fd, TIOCMGET, &mcs);
	   mcs |= TIOCM_RTS;
	   ioctl(m_fd, TIOCMSET, &mcs);

	   if (tcgetattr(m_fd, &newtio)!=0)
	   {
	      std::cerr<<"tcgetattr() 4 failed"<<std::endl;
	   }

	   //hardware handshake
	   if (hardwareHandshake)
	   {
	      newtio.c_cflag |= CRTSCTS;
	   }
	   else
	   {
	      newtio.c_cflag &= ~CRTSCTS;
	   }
	/*  if (on)
	     newtio.c_cflag |= CRTSCTS;
	  else
	     newtio.c_cflag &= ~CRTSCTS;*/
	   if (tcsetattr(m_fd, TCSANOW, &newtio)!=0)
	   {
	      std::cerr<<"tcsetattr() 2 failed"<<std::endl;
	   }

}




void Serial_Data::disconnectTTYRestore(int m_fd)
{

   if (m_fd!=-1)
   {
	tcsetattr(m_fd, TCSANOW, &m_oldtio);
	close(m_fd);
   }
   m_fd=-1;
 
}

/*
 * 函数功能：
 * 输入参数：
 * 输出参数：读取的数据字节数
 * 备    注：
 */
int  Serial_Data::readData(int m_fd,unsigned int Read_Buf_Nums)
{
  // printf("readData before\n");
   int bytesRead= read(m_fd, m_buf, Read_Buf_Nums);
  // printf("readData after\n");

   if (bytesRead<0)
   {
      std::cerr<<"read result: "<<bytesRead<<std::endl;
      perror("read: \n");
      return bytesRead ;
   }

   // if the device "disappeared", e.g. from USB, we get a read event for 0 bytes
   else if (bytesRead==0)
   {
     // disconnectTTY(); //?????
      return bytesRead ;
   }
   
   return bytesRead ;

   //const char* c=m_buf;
   
   /*
   if (m_sz!=0)
   {
//      std::cerr<<"readData() "<<bytesRead<<std::endl;
      QByteArray ba(m_buf, bytesRead);
      m_sz->writeToStdin(ba);
      return;
   }
   */


   /*
   if (m_logFile.isOpen())
   {
      m_logFile.write(m_buf, bytesRead);
   }
   */
  // char buf[16];


/*
	for(int i= 0;i< bytesRead;i++)
	{
		
		analy_dvl_data.Analy_Serial_Data(*(m_buf + i ));
		
	}
	
*/
     
	
}



/* 函数功能：
 * 输入参数： dev_name: 设备名称 ; baudrate,波特率
 * 输出参数：
 * 备    注： "/dev/ttyUSB0"
 */
int  Serial_Data::connectTTY(char * dev_name , int baudrate,string parity) 
{

   int flags= O_RDWR;  //读写
   //int flags = O_WRONLY;
   //int flags = O_RDONLY;

   int m_fd = open( dev_name, flags | O_NDELAY);
   if (m_fd<0)
   {
     // std::cerr<<"opening failed"<<std::endl;
      m_fd=-1;
      return m_fd;
   }

   // flushing is to be done after opening. This prevents first read and write to be spam'ish.
   tcflush(m_fd, TCIOFLUSH);

   //if (m_applyCb->isChecked()) 
   if(1)
   {
      int n = fcntl(m_fd, F_GETFL, 0);
      fcntl(m_fd, F_SETFL, n & ~O_NDELAY);

      if (tcgetattr(m_fd, &m_oldtio)!=0)
      {
         std::cerr<<"tcgetattr() 2 failed"<<std::endl;
      }

      //setNewOptions(9600, 8, parity, stop, softwareHandshake, hardwareHandshake);
	  //setNewOptions(m_fd , baudrate, 8, "None", "1", 0, 0);
  	    setNewOptions(m_fd , baudrate, 8, parity, "1", 0, 0);
   }
   
   return m_fd;


}




/*
 * 函数功能：
 * 输入参数：m_fd:设备id号;us_delay:us延时
 * 备    注：write函数第二个参数为指针
 */
bool Serial_Data::SendByte(int m_fd ,char c, unsigned int us_delay)
{
   if (m_fd==-1)
   {
      return false;
   }
   int res=::write(m_fd, &c, 1); //
//   std::cerr<<"wrote "<<(unsigned int)(c)<<std::endl;
   if (res<1)
   {
      std::cerr<<"write returned "<<res<<" errno: "<<errno<<std::endl;
      perror("write\n");
      return false;
   }
   usleep(us_delay); //us延时

   return true;
   
}





/*
 * 函数功能：发送字符串
 * 输入参数：m_fd:设备id号;us_delay:us延时
 * 备    注：
 */
bool Serial_Data::SendBytes(int m_fd ,char*  data, unsigned int us_delay)
{
	bool Send_falg = false;
	unsigned int i=0;
	
	while(*(data+i)!='\0')
	{
		Send_falg = SendByte(m_fd, *(data + i),us_delay); 
		i++;		
		if( Send_falg ==false )
			return false ;	
	}
	return true;	
}





/*
 * 函数功能：发送字符串
 * 输入参数：m_fd:设备id号;us_delay:us延时
 * 备    注：
 */
bool Serial_Data::SendString(int m_fd ,char*  data, int data_len ,unsigned int us_delay)
{
	bool Send_falg = false;
	unsigned int i;
	
	for(i=0;i<data_len;i++)
	{
		Send_falg = SendByte(m_fd, *(data + i),us_delay); 

		if( Send_falg ==false )
			return false ;	
	}
	return true;	
}





