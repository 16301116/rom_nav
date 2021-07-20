#ifndef  H_SERIAL_DATA_H
#define  H_SERIAL_DATA_H

#include "termios.h"
#include <unistd.h> 
#include <iostream>

using namespace std;


#define CUTECOMM_BUFSIZE (5000)

class Serial_Data
{

	public:
	
		Serial_Data();	

		char m_buf[CUTECOMM_BUFSIZE];
		struct termios m_oldtio;
	
		void setNewOptions( int m_fd , int baudrate, int databits, string parity, string stop, bool softwareHandshake, bool hardwareHandshake);
		void disconnectTTYRestore(int m_fd , bool restoreSettings);
		void disconnectTTYRestore(int m_fd);				
		int  readData(int m_fd,unsigned int Read_Buf_Nums);	
		//int  connectTTY(char * dev_name , int baudrate);
		int connectTTY(char * dev_name , int baudrate,string parity); 
		bool SendByte(int m_fd ,char c, unsigned int us_delay);
		bool SendBytes(int m_fd ,char*  data, unsigned int us_delay);
		bool SendString(int m_fd ,char*  data, int data_len ,unsigned int us_delay);
		
	protected:
	

	private:


};


#endif
