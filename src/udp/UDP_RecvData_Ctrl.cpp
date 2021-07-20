#include"udp/UDP_RecvData_Ctrl.h"


UDP_RecvData_Ctrl::UDP_RecvData_Ctrl(int recv_port)
{
	time_t currtime =time(NULL);
	tm *sins_p =localtime(&currtime);
	sprintf(file_name,"../data/output_data/control_data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	filestream = fopen(file_name,"w+");
	if (filestream==NULL)
	{
		printf("I'm controler:can't create file to save navigation data!\n");
	}
	else
	{
		printf("I'm controler:successful create file to save navigation data!\n");
	}

  const int timewait = 5;
	udprecv =  new UDP_Communication(recv_port,timewait);
	std::thread t1(Recv_Encoder_Data,this);
	t1.detach();
}



char UDP_RecvData_Ctrl::Recv_Encoder_Data(UDP_RecvData_Ctrl *ptr)
{
  static int sonnar_num = 0;
  while(1)
  {

    UDP_Communication::UDP_Info_ROV * UDP_FrameData;
		char Recv_Flag = ptr->udprecv->Recv_Decoder_ROV(&UDP_FrameData,1000);
		static unsigned char Conf_Flag  = 0x00;
		char Flag  = Recv_Flag;
		if(DECODER_NORM==Recv_Flag)  
		{
			switch(UDP_FrameData->Msg_ID) 
			{
				case NAV_Msg_ID:  //receive and no sdo
				{
					ptr->nav_data_num++;
					if(ptr->debug_flag)
					{
						printf("Recv_Flag = %d\r\n",Recv_Flag);
						printf("Control:Receive Navigation Cmd\r\n");
					}
					memcpy(&(ptr->Nav_Output_Data),UDP_FrameData->LoadData,UDP_FrameData->LoadData_Nums);
					ptr->Save_Nav_Data(&ptr->Nav_Output_Data);
					if(ptr->disp_flag&&ptr->nav_data_num/50==0)
					{
						printf("===========================Control:Receive Navigation Cmd Begin===================\n");
						printf("lon = %.6f\n",ptr->Nav_Output_Data.lon);
						printf("lat = %.6f\n",ptr->Nav_Output_Data.lat);
						printf("alt = %.6f\n",ptr->Nav_Output_Data.alt);				
						printf("x_ship = %.6f\n",ptr->Nav_Output_Data.x_ship);
						printf("y_ship = %.6f\n",	ptr->Nav_Output_Data.y_ship);
						printf("z_ship = %.6f\n",	ptr->Nav_Output_Data.z_ship);	
						printf("vel1 = %.6f\n",ptr->Nav_Output_Data.vel[0]);
						printf("vel2 = %.6f\n",	ptr->Nav_Output_Data.vel[1]);
						printf("vel3 = %.6f\n",	ptr->Nav_Output_Data.vel[2]);	
						printf("pitch = %.6f\n",ptr->Nav_Output_Data.angle[0]);
						printf("roll = %.6f\n",	ptr->Nav_Output_Data.angle[1]);
						printf("yaw = %.6f\n",	ptr->Nav_Output_Data.angle[2]);	
						printf("gyro_data_1 = %.6f\n",ptr->Nav_Output_Data.gyro_data[0]);
						printf("gyro_data_2 = %.6f\n",	ptr->Nav_Output_Data.gyro_data[1]);
						printf("gyro_data_3 = %.6f\n",	ptr->Nav_Output_Data.gyro_data[2]);	
						printf("acce_data_1 = %.6f\n",ptr->Nav_Output_Data.acce_data[0]);
						printf("acce_data_2 = %.6f\n",	ptr->Nav_Output_Data.acce_data[1]);
						printf("acce_data_3 = %.6f\n",	ptr->Nav_Output_Data.acce_data[2]);	
						printf("timestamp = %.6f\n",	ptr->Nav_Output_Data.timestamp);	
						printf("===========================Control:Receive Navigation Cmd End===================\n");
					}
					ptr->nav_que.push(ptr->Nav_Output_Data);
					if(ptr->nav_que.size()>2)
					{
						Nav_Output_Info data_buf;
						ptr->nav_que.try_pop(data_buf);				
					}
					break;
				}
				
				default:
				{
					break;
				}

			}
				 
		}         

		if(DECODER_NORM==Recv_Flag||DECODER_ERROR==Recv_Flag)
		{

		}  
	usleep(10000); //100HZ	
		
	}

}


void UDP_RecvData_Ctrl::Save_Nav_Data(Nav_Output_Info *data)
{
	static int counter_num = 0;
  	time_t timep;
  	struct tm *p;
  	time(&timep);
  	p = gmtime(&timep);
	fprintf(filestream,"%04d-",1900+p->tm_year);
  	fprintf(filestream,"%02d-",1+p->tm_mon);
  	fprintf(filestream,"%02d-",p->tm_mday);
  	fprintf(filestream,"%02d-",p->tm_hour);
  	fprintf(filestream,"%02d-",p->tm_min);
  	fprintf(filestream,"%02d ",p->tm_sec);
	fprintf(filestream,"%.6f ",(float)data->lon); 
	fprintf(filestream,"%.6f ",(float)data->lat); 
	fprintf(filestream,"%.6f ",(float)data->alt); 
	fprintf(filestream,"%.6f ",(float)data->x_ship); 
	fprintf(filestream,"%.6f ",(float)data->y_ship);
	fprintf(filestream,"%.6f ",(float)data->z_ship);	 
	fprintf(filestream,"%.6f ",data->vel[0]); 
	fprintf(filestream,"%.6f ",data->vel[1]); 
	fprintf(filestream,"%.6f ",data->vel[2]); 
	fprintf(filestream,"%.6f ",data->angle[0]); 
	fprintf(filestream,"%.6f ",data->angle[1]); 
	fprintf(filestream,"%.6f ",data->angle[2]); 
	fprintf(filestream,"%.6f ",data->gyro_data[0]); 
	fprintf(filestream,"%.6f ",data->gyro_data[1]); 
	fprintf(filestream,"%.6f ",data->gyro_data[2]); 
	fprintf(filestream,"%.6f ",data->acce_data[0]); 
	fprintf(filestream,"%.6f ",data->acce_data[1]); 
	fprintf(filestream,"%.6f ",data->acce_data[2]); 
	fprintf(filestream,"%.6f ",data->timestamp); 
	for(int i =0;i<13;i++)
	{
		fprintf(filestream,"%d ",data->packup_char[i]); 
	}
	fprintf(filestream,"\n"); 

	counter_num++;
	if(counter_num==50)
	{
		counter_num = 0;
		fflush(filestream);
	}
	fflush(filestream);
}



