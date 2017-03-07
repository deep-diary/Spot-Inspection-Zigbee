#ifndef _GPS_H_
#define _GPS_H_
#include "SerialApp.h"

#ifdef ZIGBEE_GPS
#include "Hal_types.h"

#define LITHIUM_NUMS    1       //1节电池
#define SAMPLE_RATE     2       //1/2采样
#define SOFTWARE_VERSION     "170228"       //软件版本
#define HARDWARE_VERSION     "170224"       //硬件版本

#define GPS_MAX_SVS 32
#define NMEA_MAX_SV_INFO    4

//调试宏
#define GPS_DEBUG       1

#define LITHIUM_NUMS    2       //1节电池
#define SAMPLE_RATE     3       //1/2采样

//每个卫星信息
typedef struct {
    //编号
    uint8     prn;
    //是否定位 true:此卫星用于定位
    uint8 isValid;
    //信噪比
    int   snr;
    //仰角
    float   elevation;
    //方位角
    float   azimuth;
}GpsSvInfo;


typedef struct _time_
{
	uint8 year;
	uint8 month;
	uint8 day;
	uint8 hour;
	uint8 min;
	uint8 sec;
}TIME;

typedef struct _gps_info_
{
	uint8 state;//状态 0没信号 1已经定位 2正在定位
	uint8 planet;//卫星数量
	uint8 usePlanet;//使用卫星数量
	uint8 fix;  //0:未定位　1:2D定位　2:3D定位
	double longitude;//经度
	double latitude;//纬度
	double altitude;//海拔
	uint8 speed;//速度
	double direction;//航向角 以正北为0 顺时针360度
	double hdop;    //水平精度
	TIME UTC_time; 
	uint8 SorN;  //S(南半球) is 0 N(北半球) is 1
	uint8 EorW;  //E(东经) is 1 W(西经) is 0
	uint8 valid; //TRUE:此数据有效

	//卫星信息表
	int sv_count;
	GpsSvInfo sv_list[GPS_MAX_SVS];
}GpsInfo;


void  SendGpsDataToCoor();
void GpsDataParse(char* buff, int mLen);
uint8 GetYear();
uint8 GetMonth();
uint8 GetDay();
uint8 GetHour();
uint8 GetMinute();
uint8 GetSecond();
uint8 GetSorN();
uint8 GetEorW();
double GetLongitude();
double GetLatitude();
uint8 GetSpeed();
double GetDirection();
double GetAltitude();
uint8 GetFix();
void GpsInit();
double GetLongitude1();
double GetLatitude1();
extern void  SendGpsDataToCoor();
extern void gpsDisplay(uint8* gpsData);
extern void gpsInit(void);
#endif
#endif