#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "Hal_uart.h"
#include "Hal_types.h"
#include "hal_lcd.h"
#include "OnBoard.h"
#include "gps.h"
#include "gprs.h"
#include "CattleOrientation.h"
#include "SerialApp.h"
#ifdef ZIGBEE_GPS

extern uint16 HalUARTWrite(uint8 port, uint8 *buf, uint16 len);

static char frameData[100]; //每一帧要解析的数据
static unsigned char frameDataLen; //上面帧数据的长度
GpsInfo m_Info;

void gpsInit(void)
{
  dev_type=DEV_TYPE_GPS;
  memset(ep_addr.hard_vers,0,MAX_BYTES_OF_VERS_INFO);
  memset(ep_addr.soft_vers,0,MAX_BYTES_OF_VERS_INFO);
  strcpy(ep_addr.hard_vers,HARDWARE_VERSION);
  strcpy(ep_addr.soft_vers,SOFTWARE_VERSION);
  HalUARTWrite(0, "AT+MGPSC=1\r", osal_strlen("AT+MGPSC=1\r"));
  
}
void gpsGet(void)
{
  HalUARTWrite(0, "AT+GETGPS=\"ALL\"\r", osal_strlen("AT+GETGPS=\"ALL\"\r")); 
  serial_type = SERIAL_TYPE_GPS_DATA;
}
void CalibrateTime()
{
	uint8 year=m_Info.UTC_time.year;
	uint8 month=m_Info.UTC_time.month;
	uint8 day=m_Info.UTC_time.day;
	uint8 hour=m_Info.UTC_time.hour;

	hour+=8;
	if(hour>23)
	{
		hour=hour%24;
		day++;

		switch(month)
		{
			case 1:
			case 3:
			case 5:
			case 7:
			case 8:
			case 10:
			{
				if(day>31)
				{
					day=1;
					month++;
				}
			}
			break;
			
			case 2:
			{
				if(year==16||year==20||year==24||year==28||year==32||year==36||year==40)
				{
					if (day>29)
					{
						day=1;
						month++;
					}
				}
				else
				{
					if(day>28)
					{
						day=1;
						month++;
					}
				}
			}
			break;

			case 4:
			case 6:
			case 9:
			case 11:
			{
				if(day>30)
				{
					day=1;
					month++;
				}
			}
			break;
			
			case 12:
			{
				if(day>31)
				{
					day=1;
					month=1;
					year++;
				}
			}
			break;
				
		}
	}

	m_Info.UTC_time.year=year;
	m_Info.UTC_time.month=month;
	m_Info.UTC_time.day=day;
	m_Info.UTC_time.hour=hour;
}

void ParseframeData(char* frame, int bufflen)
{
	//对buff的数据进行解析,解析出各个','号之间的数据放在keyBuff里
	int keyBuffIndex=0;
	int idx=0;
	char keyBuff[30][12];
	int i=0;
//    char printfBuff[30]={0};

	if(bufflen<=0) return;

	osal_memset(keyBuff, 0, sizeof(keyBuff));
	for(i=0; i<bufflen; i++)
	{
		if(frame[i]==',')
		{
			//下标++
			keyBuffIndex++;
			idx=0;
		}
		else if(frame[i]=='\r' || frame[i]=='\n')
		{
		}
		else
		{
			keyBuff[keyBuffIndex][idx]=frame[i];
			idx++;
		}
	}
	
	if(0==strcmp( &keyBuff[0][3], "GSV" )) //获得卫星数量
	{
		//$GPGSV，(1)，(2)，(3)，(4)，(5)，(6)，(7)，…(4),(5)，(6)，(7)*hh(CR)(LF)
		//(1)总的GSV语句电文数；2;
		//(2)当前GSV语句号:1;
		//(3)可视卫星总数:08;
		//(4)PRN码（伪随机噪声码）　也可以认为是卫星编号
		//(5)仰角(00～90度):33度;
		//(6)方位角(000～359度):240度;
		//(7)信噪比(00～99dB):


		//未注册时信息如"$GPGSV,1,1,00*79",此时根本没有卫星数。
		if(keyBuffIndex<4) return;

        int num = atoi(keyBuff[1]);//gsv语句总条数
        int seq = atoi(keyBuff[2]);//当前gsv是第几条
        int cnt = atoi(keyBuff[3]);//卫星数
        int sv_base = (seq - 1)*NMEA_MAX_SV_INFO;
        int sv_num = cnt - sv_base;
		int idx, base = 4, base_idx;

		if (sv_num > NMEA_MAX_SV_INFO)
		{
			sv_num = NMEA_MAX_SV_INFO;
		}
		
		m_Info.planet=cnt;

        for (idx = 0; idx < sv_num; idx++)
		{            
			base_idx = base*(idx+1);
			int id    = atoi(keyBuff[base_idx+0]);
			float ele = atof(keyBuff[base_idx+1]);
			float azi = atof(keyBuff[base_idx+2]);
			int snr = atoi(keyBuff[base_idx+3]);

			//检测一下是否有重复的
			for(i=0; i<m_Info.sv_count && i<GPS_MAX_SVS; i++)
			{
				if(m_Info.sv_list[i].prn==id)
				{
					m_Info.sv_list[i].prn=id;
					m_Info.sv_list[i].snr=snr;
					m_Info.sv_list[i].elevation=ele;
					m_Info.sv_list[i].azimuth=azi; 
					break;
				}  
			}

			//没有重复就增加
			if(i==m_Info.sv_count)
			{
				if(i<GPS_MAX_SVS)
				{
					m_Info.sv_list[i].prn=id;
					m_Info.sv_list[i].snr=snr;
					m_Info.sv_list[i].elevation=ele;
					m_Info.sv_list[i].azimuth=azi;
					m_Info.sv_count++;
				}
				else
				{
				}
			}
        }
	}
	else if(0==strcmp( &keyBuff[0][3], "GSA" )) //是否型式  2D or 3D
	{
		//$GPGSA,<1>,<2>,<3>,<3>,,,,,<3>,<3>,<3>,<4>,<5>,<6>,<7><CR><LF> 
		//1)模式 2：M = 手动， A = 自动。 
		//2)模式 1：定位型式 1 = 未定位， 2 = 二维定位， 3 = 三维定位。 
		//3) PRN 数字：01 至 32 表天空使用中的卫星编号，最多可接收12颗卫星信息。
		//4) PDOP-位置精度稀释 0.5 至 99.9. 
		//5) HDOP-水平精度稀释 0.5 to 99.9. 
		//6) VDOP-垂直精度稀释 0.5 to 99.9. 
		//7) Checksum.(检查位).


		//0:未定位　1:2D定位　2:3D定位
		if(keyBuff[2][0]=='2')
		{
			m_Info.fix=1;
		}
		else if(keyBuff[2][0]=='3')
		{
			m_Info.fix=2;
		}
		else
		{
			m_Info.fix=0;
		}


		//复位为未定位状态
		for(i=0; i<GPS_MAX_SVS; i++)
		{
            m_Info.sv_list[i].isValid=FALSE;
		}

		//已经定位上
		if(m_Info.fix==1 || m_Info.fix==2)
		{
			int idx, max = 12; 

			for(idx=0; idx<max; idx++)
			{
				//已经没有
				if(keyBuff[idx+3][0]==0)
				{
					break;
				}

				//定位的卫星号
				int id = atoi(keyBuff[idx+3]);


				//检测一下是否有重复的
				for(i=0; i<m_Info.sv_count && i<GPS_MAX_SVS; i++)
				{
					if(m_Info.sv_list[i].prn==id)
					{
						m_Info.sv_list[i].prn=id;
						m_Info.sv_list[i].isValid=TRUE;
						break;
					}  
				}

				//没有重复就增加
				if(i==m_Info.sv_count)
				{
					if(i<GPS_MAX_SVS)
					{
						m_Info.sv_list[i].prn=id;
						m_Info.sv_list[i].isValid=TRUE;
						m_Info.sv_count++;
					}
					else
					{
					}
				}
			}
		}
	}
	else if(0==strcmp( &keyBuff[0][3], "RMC" )) //取时间
	{
		//$GPRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11><CR><LF>			
		//1) 标准定位时间（UTC time）格式：时时分分秒秒.秒秒秒（hhmmss.sss）。 
		//2) 定位状态，A = 数据可用，V = 数据不可用。 
		//3) 纬度，格式：度度分分.分分分分（ddmm.mmmm）。 
		//4) 纬度区分，北半球（N）或南半球（S）。 
		//5) 经度，格式：度度分分.分分分分。
		//6) 经度区分，东（E）半球或西（W）半球。 
		//7) 相对位移速度， 0.0 至 1851.8 knots 
		//8) 相对位移方向，000.0 至 359.9度。实际值。 
		//9) 日期，格式：日日月月年年（ddmmyy）。 
		//10) 磁极变量，000.0 至180.0。 
		//11) 度数。 
		//12) Checksum.(检查位)


          //cattleOrientationReport();  

		//数据有效就保存
		if(keyBuff[2][0]=='A')
		{
			//处理时间
			char year[3]={0}, month[3]={0}, day[3]={0};
			char hour[3]={0}, min[3]={0}, sec[3]={0};

			//时间
			memcpy( (void *)hour, (const void *)(&keyBuff[1][0]), 2 );
			memcpy( (void *)min, (const void *)(&keyBuff[1][2]), 2 );
			memcpy( (void *)sec, (const void *)(&keyBuff[1][4]), 2 );

			m_Info.UTC_time.hour=atoi(hour);
			m_Info.UTC_time.min=atoi(min);
			m_Info.UTC_time.sec=atoi(sec);

			//日期
			memcpy( (void *)day, (const void *)(&keyBuff[9][0]), 2 );
			memcpy( (void *)month, (const void *)(&keyBuff[9][2]), 2 );
			memcpy( (void *)year, (const void *)(&keyBuff[9][4]), 2 );

			m_Info.UTC_time.year=atoi(year);
			m_Info.UTC_time.month=atoi(month);
			m_Info.UTC_time.day=atoi(day);

			//+8为北京北京时间
			CalibrateTime();		
		}
                        //上传数据到协调器
          
            //SendGpsDataToCoor();
          if(is_time_to_report)
          {
            HalLcdWriteString("getting gps.....", HAL_LCD_LINE_4 ); 
            if(m_Info.fix==1 || m_Info.fix==2)
            {
              SendGpsDataToCoor();
              HalLcdWriteString("got the gps.....", HAL_LCD_LINE_4 ); 
              cattleOrientationReport();
              is_time_to_report = 0;
              GPS_POWER = 0;    
              m_Info.fix=0;   //初始化为未定位状态
            }
  
          }
	}
	else if(0==strcmp( &keyBuff[0][3], "GGA" ))//获得GPGGA经纬海拔
	{
		//$GPGGA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,M,<10>,M,<11>,<12>*xx<CR><LF>
		//<1> UTC 时间，格式为hhmmss.sss；
		//<2> 纬度，格式为ddmm.mmmm(第一位是零也将传送)；
		//<3> 纬度半球，N 或S(北纬或南纬)
		//<4> 经度，格式为dddmm.mmmm(第一位零也将传送)；
		//<5> 经度半球，E 或W(东经或西经)
		//<6> 定位质量指示，0=定位无效，1=定位有效；
		//<7>使用卫星数量，从00到12(第一个零也将传送)
		//<8>水平精确度hdop，0.5到99.9
		//<9>天线离海平面的高度，-9999.9到9999.9米M指单位米
		//<10>大地水准面高度，-9999.9到9999.9米M指单位米
		//<11>差分GPS数据期限(RTCMSC-104)，最后设立RTCM传送的秒数量
		//<12>差分参考基站标号，从0000到1023(首位0也将传送)。

        if(GPS_DEBUG)
        {
   //         HalUARTWrite(0, frame, bufflen);
   //         HalUARTWrite(0, "\r\n", 2);

   //         sprintf(printfBuff, "%s\r\n",keyBuff[2]);
   //         HalUARTWrite(0, printfBuff, osal_strlen(printfBuff));

   //         sprintf(printfBuff, "%s\r\n",keyBuff[4]);
   //         HalUARTWrite(0, printfBuff, osal_strlen(printfBuff));
            
        }
		
		m_Info.latitude=atof(keyBuff[2]);//纬度

		//纬度半球
		if(keyBuff[3][0]=='S')
		{
			m_Info.SorN=0;
		}
		else if(keyBuff[3][0]=='N')
		{
			m_Info.SorN=1;
		}
		else
		{
			m_Info.valid=FALSE;
		}

		//经度
		m_Info.longitude=atof(keyBuff[4]);//经度

        if(GPS_DEBUG)
        {
//            sprintf(printfBuff, "%f\r\n",m_Info.latitude);
//            HalUARTWrite(0, printfBuff, osal_strlen(printfBuff));            

//            sprintf(printfBuff, "%f\r\n",m_Info.longitude);
//            HalUARTWrite(0, printfBuff, osal_strlen(printfBuff));            
        }


		//经度半球
		if(keyBuff[5][0]=='E')
		{
			m_Info.EorW=1;
		}
		else if(keyBuff[5][0]=='W')
		{
			m_Info.EorW=0;
		}
		else
		{
			m_Info.valid=FALSE;
		}
		
		//GPS 状态 0:未定位, 1非差分定位,2差分定位,3正在估算
		if(keyBuff[6][0]=='1' || keyBuff[6][0]=='2')//定位
		{
			m_Info.state=1;
		}
		else if(keyBuff[6][0]=='6')//正在估算
		{
			m_Info.state=2;
		}
		else//未定位
		{
			m_Info.state=0;
		}

		m_Info.usePlanet=(uint8)atof(keyBuff[7]);//使用的卫星数据		
		m_Info.hdop=atof(keyBuff[8]);//水平精度
		m_Info.altitude=atof(keyBuff[9]);//海拔
	}
	else if(0==strcmp( &keyBuff[0][3], "VTG" ))//取速度和方向
	{
		//$GPVTG,<1>,T,<2>,M,<3>,N,<4>,K,<5>*hh			
		//<1> 以正北为参考基准的地面航向(000~359度，前面的0也将被传输)
		//<2> 以磁北为参考基准的地面航向(000~359度，前面的0也将被传输)
		//<3> 地面速率(000.0~999.9节，前面的0也将被传输)
		//<4> 地面速率(0000.0~1851.8公里/小时，前面的0也将被传输)
		//<5> 模式指示(仅NMEA0183 3.00版本输出，A=自主定位，D=差分，E=估算，N=数据无效

		//方向
		m_Info.direction=atof(keyBuff[1]);

		//速度
		m_Info.speed=(uint16)atof(keyBuff[7]);
	}
}

void GpsDataParse(char* buff, int mLen)
{
	//找出gps的一帧数据,以'$'开头，以"\r\n"结束的
	for(int i=0; i<mLen; i++)
	{
		if(buff[i]=='$' )
		{
			if(frameDataLen>0)
			{
				ParseframeData(frameData, frameDataLen);
			}
			
			memset(frameData, 0, sizeof(frameData));
			frameDataLen=0;

			frameData[frameDataLen]=buff[i];
			frameDataLen++;
		}
		else if(buff[i]=='\r')
		{
            //如果找到一帧数据，就开始解析
			if(frameDataLen>0) ParseframeData(frameData, frameDataLen);

			memset(frameData, 0, sizeof(frameData));
			frameDataLen=0;
		}
		else if(buff[i]=='\n')
		{
            //如果找到一帧数据，就开始解析
			if(frameDataLen>0) ParseframeData(frameData, frameDataLen);
			
			memset(frameData, 0, sizeof(frameData));
			frameDataLen=0;
		}
		else
		{
			frameData[frameDataLen]=buff[i];
			frameDataLen++;	
		}
	}

}

uint8 GetYear()
{
    return m_Info.UTC_time.year;
}

uint8 GetMonth()
{
    return m_Info.UTC_time.month;
}

uint8 GetDay()
{
    return m_Info.UTC_time.day;
}

uint8 GetHour()
{
    
    return m_Info.UTC_time.hour;
}

uint8 GetMinute()
{
    return m_Info.UTC_time.min;
}

uint8 GetSecond()
{
    return m_Info.UTC_time.sec;
}

uint8 GetSorN()
{
    return m_Info.SorN;
}

uint8 GetEorW()
{
    return m_Info.EorW;
}

double GetLongitude()
{
    double longitude=m_Info.longitude/100;
    uint16 longitude1=(uint16)longitude;
    double longitude2=longitude-longitude1;
    return longitude1+(longitude2*100)/60;
}

double GetLongitude1()
{
    return m_Info.longitude;
}
double GetLatitude1()
{
    return m_Info.latitude;
}

double GetLatitude()
{
    double latitude=m_Info.latitude/100;
    uint16 latitude1=(uint16)latitude;
    double latitude2=latitude-latitude1;
    return latitude1+(latitude2*100)/60;
}

uint8 GetSpeed()
{
    return m_Info.speed;
}

double GetDirection()
{
    return m_Info.direction;
}

double GetAltitude()
{
    return m_Info.altitude;
}

uint8 GetFix()
{
    if(1==m_Info.fix || 2==m_Info.fix) return 1;

    return 0;
}

void GpsInit()
{
    m_Info.fix=0;
    m_Info.SorN=1; 
	m_Info.EorW=1; 
}

//----------------------------将gps数据发送给协调器------------------------
void  SendGpsDataToCoor()
{
	uint8 data[30]={0};
	uint32 longitude =(uint32)(GetLongitude()*10000000);
	uint32 latitude =(uint32)(GetLatitude()*10000000);
	uint16 speed =(uint16)(GetSpeed());
	uint16 direction=(uint16)(GetDirection());
	uint16 altitude =(uint16)GetAltitude();


	//是否定位
	data[0]=GetFix();

    //时间
	data[1] = GetYear();
	data[2] = GetMonth();
	data[3] = GetDay();
	data[4] = GetHour();
	data[5] = GetMinute();
	data[6] = GetSecond();

	//经纬度标志
	data[7]=0x00;
	data[7]|=GetSorN()?1:0;
	data[7]|=GetEorW()?2:0;

	//经度
	data[8]=(uint8)(longitude&0xff);
	data[9]=(uint8)((longitude>>8)&0xff);
	data[10]=(uint8)((longitude>>16)&0xff);
	data[11]=(uint8)((longitude>>24)&0xff);

	//纬度
	data[12]=(uint8)(latitude&0xff);
	data[13]=(uint8)((latitude>>8)&0xff);
	data[14]=(uint8)((latitude>>16)&0xff);
	data[15]=(uint8)((latitude>>24)&0xff);

	//速度
	data[16]=(uint8)(speed&0xff);
	data[17]=(uint8)((speed>>8)&0xff);

	//方向
	data[18]=(uint8)(direction&0xff);
	data[19]=(uint8)((direction>>8)&0xff);

	//海拔
	data[20]=(uint8)(altitude&0xff);
	data[21]=(uint8)((altitude>>8)&0xff);

    //LCD显示
    gpsDisplay(data);
    //把GPS数据发到协调器
    //SerialApp_SendDataToCoordinator(data, 22,SERIALAPP_GPS_PKG);
    
}

void gpsSave(P_GpsOutData p_gps_data)
{
  uint8 location=0x00;
  p_gps_data->fix=GetFix();
  p_gps_data->UTC_time.year=GetYear();
  p_gps_data->UTC_time.month=GetMonth();
  p_gps_data->UTC_time.day=GetDay();
  p_gps_data->UTC_time.hour=GetHour();
  p_gps_data->UTC_time.min=GetMinute();
  p_gps_data->UTC_time.sec=GetSecond();
  
  location|=GetSorN()?1:0;
  location|=GetEorW()?2:0;
  
  p_gps_data->location=location;
  p_gps_data->longitude=(uint32)(GetLongitude()*10000000);
  p_gps_data->latitude=(uint32)(GetLatitude()*10000000);
  p_gps_data->speed=(uint16)(GetSpeed());
  p_gps_data->direction=(uint16)(GetDirection());
  p_gps_data->altitude=(uint16)GetAltitude();  
  
  p_gps_data->longitude = BigLittleSwap32(p_gps_data->longitude);
  p_gps_data->latitude = BigLittleSwap32(p_gps_data->latitude);
  p_gps_data->speed = BigLittleSwap16(p_gps_data->speed);
  p_gps_data->direction = BigLittleSwap16(p_gps_data->direction);
  p_gps_data->altitude = BigLittleSwap16(p_gps_data->altitude);
}
#endif

//-------------------------------显示gps信息
void gpsDisplay(uint8* gpsData)
{
    char buff[50]={0};

    if(gpsData==0) return;
    
    if(gpsData[0]>0)
    {
        //
        sprintf(buff, "  20%02d年%02d月%02d日",gpsData[1],gpsData[2],gpsData[3]);
        HalLcdWriteString( buff, HAL_LCD_LINE_1 );  

        sprintf(buff, "  %02d时%02d分%02d秒",gpsData[4],gpsData[5],gpsData[6]);
        HalLcdWriteString( buff, HAL_LCD_LINE_2 );  


        unsigned long bb=BUILD_UINT32(gpsData[8],gpsData[9],gpsData[10],gpsData[11]);
        double longitude=((double)bb)/10000000.0;        
        bb=BUILD_UINT32(gpsData[12],gpsData[13],gpsData[14],gpsData[15]);
        double latitude=((double)bb)/10000000.0;

        if((gpsData[7]&0x01)==0x01)
        {
            //北半球

            if((gpsData[7]&0x02)==0x02)
            {
                //东经
                sprintf(buff, "北纬:%f",latitude);
//                HalUARTWrite(0, (uint8 *)buff, osal_strlen(buff)); 
//                HalUARTWrite(0, "\n", 1); 
                HalLcdWriteString( buff, HAL_LCD_LINE_3 );  
                sprintf(buff, "东经:%f",longitude);
//                HalUARTWrite(0, (uint8 *)buff, osal_strlen(buff)); 
//                HalUARTWrite(0, "\n", 1); 
                HalLcdWriteString( buff, HAL_LCD_LINE_4 );  
            }
            else
            {
                //西经
                sprintf(buff, "北纬:%f",latitude);
                HalLcdWriteString( buff, HAL_LCD_LINE_3 );  
                sprintf(buff, "东经:%f",longitude);                
                HalLcdWriteString( buff, HAL_LCD_LINE_4 );  
            }
        }
        else
        {
            //南半球
            //if(GetEorW()>0)
            if((gpsData[7]&0x02)==0x02)
            {
                //东经
                sprintf(buff, "南纬:%f",latitude);
                HalLcdWriteString( buff, HAL_LCD_LINE_3 );  
                sprintf(buff, "东经:%f",longitude);
                HalLcdWriteString( buff, HAL_LCD_LINE_4 );  
            }
            else
            {
                //西经
                sprintf(buff, "南纬:%f",latitude);
                HalLcdWriteString( buff, HAL_LCD_LINE_3 );  
                sprintf(buff, "西经:%f",longitude);                
                HalLcdWriteString( buff, HAL_LCD_LINE_4 );  
            }
        }        
    }
    else
    {
        HalLcdWriteString( "GPS未定位", HAL_LCD_LINE_1 );  
    }      
}

