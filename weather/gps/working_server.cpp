#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

//WiFi配置
const char* ssid = "zzc";        // 替换为你的WiFi名称
const char* password = "12345678";  // 替换为你的WiFi密码

//服务器配置
const char* server = "http://3.10.54.50:8000/api/cart/location/";

//此处为了兼容其他的多串口Arduino板子
int L = 13; //LED指示灯引脚 

#define GpsSerial Serial2
#define DebugSerial Serial

// ESP32 TX2 = GPIO17, RX2 = GPIO16

struct
{
	char GPS_Buffer[80];
	bool isGetData;		//是否获取到GPS数据
	bool isParseData;	//是否解析完成
	char UTCTime[11];		//UTC时间
	char latitude[11];		//纬度
	char N_S[2];		//N/S
	char longitude[12];		//经度
	char E_W[2];		//E/W
	bool isUsefull;		//定位信息是否有效
} Save_Data;

const unsigned int gpsRxBufferLength = 600;
char gpsRxBuffer[gpsRxBufferLength];
unsigned int ii = 0;

// 时间控制变量
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000; // 5秒间隔

// WiFi和HTTP客户端
WiFiClient client;
HTTPClient http;

void errorLog(int num)
{
	DebugSerial.print("ERROR");
	DebugSerial.println(num);
	while (1)
	{
		digitalWrite(L, HIGH);
		delay(300);
		digitalWrite(L, LOW);
		delay(300);
	}
}

void clrGpsRxBuffer(void)
{
	memset(gpsRxBuffer, 0, gpsRxBufferLength);      //清空
	ii = 0;
}

void connectWiFi()
{
	DebugSerial.println("Connecting to WiFi...");
	WiFi.begin(ssid, password);
	
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		DebugSerial.print(".");
	}
	
	DebugSerial.println("");
	DebugSerial.println("WiFi connected!");
	DebugSerial.print("IP address: ");
	DebugSerial.println(WiFi.localIP());
}

void sendGPSData()
{
	// 使用固定的假坐标进行测试
	String fake_latitude = "51.9";   // 北京天安门纬度
	String fake_longitude = "-0.4"; // 北京天安门经度
	
	if (WiFi.status() == WL_CONNECTED)
	{
		// 构造POST数据 - 使用固定坐标
		String postData = "id=imperial.ac.uk&lat=" + fake_latitude + "&lng=" + fake_longitude;
// 应该发送: id=imperial.ac.uk&lat=39.9042&lng=116.4074
		
		DebugSerial.println("Sending  GPS data to server...");
		DebugSerial.println("Data: " + postData);
		
		http.begin(client, server);
		http.addHeader("Content-Type", "application/x-www-form-urlencoded");
		
		int httpResponseCode = http.POST(postData);
		
		if (httpResponseCode > 0) {
			String response = http.getString();
			DebugSerial.println("HTTP Response code: " + String(httpResponseCode));
			DebugSerial.println("Response: " + response);
			
			if (httpResponseCode == 200) {
				DebugSerial.println("GPS data sent successfully!");
				// 成功发送时LED快闪一次
				digitalWrite(L, HIGH);
				delay(100);
				digitalWrite(L, LOW);
			}
		} else {
			DebugSerial.println("Error sending data. HTTP Response code: " + String(httpResponseCode));
		}
		
		http.end();
	}
	else
	{
		DebugSerial.println("WiFi not connected, skipping send");
	}

	/* 注释掉原先基于GPS传感器的发送逻辑
	if (Save_Data.isUsefull && WiFi.status() == WL_CONNECTED)
	{
		// 构造POST数据
		String postData = "gps=" + String(Save_Data.latitude) + "," + String(Save_Data.longitude);
		
		DebugSerial.println("Sending GPS data to server...");
		DebugSerial.println("Data: " + postData);
		
		http.begin(client, server);
		http.addHeader("Content-Type", "application/x-www-form-urlencoded");
		
		int httpResponseCode = http.POST(postData);
		
		if (httpResponseCode > 0) {
			String response = http.getString();
			DebugSerial.println("HTTP Response code: " + String(httpResponseCode));
			DebugSerial.println("Response: " + response);
			
			if (httpResponseCode == 200) {
				DebugSerial.println("GPS data sent successfully!");
				// 成功发送时LED快闪一次
				digitalWrite(L, HIGH);
				delay(100);
				digitalWrite(L, LOW);
			}
		} else {
			DebugSerial.println("Error sending data. HTTP Response code: " + String(httpResponseCode));
		}
		
		http.end();
	}
	else if (!Save_Data.isUsefull)
	{
		DebugSerial.println("GPS data not valid, skipping send");
	}
	else if (WiFi.status() != WL_CONNECTED)
	{
		DebugSerial.println("WiFi not connected, skipping send");
	}
	*/
}

void printGpsBuffer()
{
	if (Save_Data.isParseData)
	{
		Save_Data.isParseData = false;
		
		DebugSerial.print("Save_Data.UTCTime = ");
		DebugSerial.println(Save_Data.UTCTime);

		if(Save_Data.isUsefull)
		{
			DebugSerial.print("Save_Data.latitude = ");
			DebugSerial.println(Save_Data.latitude);
			DebugSerial.print("Save_Data.N_S = ");
			DebugSerial.println(Save_Data.N_S);
			DebugSerial.print("Save_Data.longitude = ");
			DebugSerial.println(Save_Data.longitude);
			DebugSerial.print("Save_Data.E_W = ");
			DebugSerial.println(Save_Data.E_W);
			
			/* 注释掉基于GPS传感器数据的发送触发
			// 检查是否到了发送时间
			unsigned long currentTime = millis();
			if (currentTime - lastSendTime >= sendInterval) {
				sendGPSData();
				lastSendTime = currentTime;
			}
			*/
		}
		else
		{
			DebugSerial.println("GPS DATA is not usefull!");
		}
	}
}

void parseGpsBuffer()
{
	char *subString;
	char *subStringNext;
	if (Save_Data.isGetData)
	{
		Save_Data.isGetData = false;
		DebugSerial.println("**************");
		DebugSerial.println(Save_Data.GPS_Buffer);

		// 清空之前的数据
		memset(Save_Data.UTCTime, 0, sizeof(Save_Data.UTCTime));
		memset(Save_Data.latitude, 0, sizeof(Save_Data.latitude));
		memset(Save_Data.N_S, 0, sizeof(Save_Data.N_S));
		memset(Save_Data.longitude, 0, sizeof(Save_Data.longitude));
		memset(Save_Data.E_W, 0, sizeof(Save_Data.E_W));
		
		for (int i = 0 ; i <= 6 ; i++)
		{
			if (i == 0)
			{
				if ((subString = strstr(Save_Data.GPS_Buffer, ",")) == NULL)
					errorLog(1);	//解析错误
			}
			else
			{
				subString++;
				if ((subStringNext = strstr(subString, ",")) != NULL)
				{
					char usefullBuffer[2] = {0}; 
					switch(i)
					{
						case 1:memcpy(Save_Data.UTCTime, subString, subStringNext - subString);break;	//获取UTC时间
						case 2:memcpy(usefullBuffer, subString, subStringNext - subString);break;	//获取定位状态
						case 3:memcpy(Save_Data.latitude, subString, subStringNext - subString);break;	//获取纬度信息
						case 4:memcpy(Save_Data.N_S, subString, subStringNext - subString);break;	//获取N/S
						case 5:memcpy(Save_Data.longitude, subString, subStringNext - subString);break;	//获取经度信息
						case 6:memcpy(Save_Data.E_W, subString, subStringNext - subString);break;	//获取E/W

						default:break;
					}

					subString = subStringNext;
					Save_Data.isParseData = true;
					if(usefullBuffer[0] == 'A')
						Save_Data.isUsefull = true;
					else if(usefullBuffer[0] == 'V')
						Save_Data.isUsefull = false;

				}
				else
				{
					errorLog(2);	//解析错误
				}
			}
		}
	}
}

void gpsRead() {
	while (GpsSerial.available())
	{
		gpsRxBuffer[ii++] = GpsSerial.read();
		if (ii == gpsRxBufferLength)clrGpsRxBuffer();
	}

	char* GPS_BufferHead;
	char* GPS_BufferTail;
	if ((GPS_BufferHead = strstr(gpsRxBuffer, "$GPRMC,")) != NULL || (GPS_BufferHead = strstr(gpsRxBuffer, "$GNRMC,")) != NULL )
	{
		if (((GPS_BufferTail = strstr(GPS_BufferHead, "\r\n")) != NULL) && (GPS_BufferTail > GPS_BufferHead))
		{
			memcpy(Save_Data.GPS_Buffer, GPS_BufferHead, GPS_BufferTail - GPS_BufferHead);
			Save_Data.isGetData = true;
 
			clrGpsRxBuffer();
		}
	}
}

void setup()	//初始化内容
{
	pinMode(L, OUTPUT);
	digitalWrite(L, LOW);
	
	GpsSerial.begin(115200, SERIAL_8N1, 25, 26); // TX <---> pin 25 of esp32
	DebugSerial.begin(115200);
	DebugSerial.println("ILoveMCU.taobao.com");
	DebugSerial.println("Starting GPS with Server Upload...");

	// 初始化WiFi
	connectWiFi();

	Save_Data.isGetData = false;
	Save_Data.isParseData = false;
	Save_Data.isUsefull = false;
	
	lastSendTime = millis();
	
	DebugSerial.println("Setup complete, waiting for GPS data...");
}

void loop()		//主循环
{
	// 检查WiFi连接状态
	if (WiFi.status() != WL_CONNECTED) {
		DebugSerial.println("WiFi disconnected, attempting to reconnect...");
		connectWiFi();
	}
	
	/* 注释掉GPS传感器相关处理，使用固定坐标测试
	gpsRead();	//获取GPS数据
	parseGpsBuffer();//解析GPS数据
	printGpsBuffer();//输出解析后的数据
	*/
	
	// 使用固定坐标，每5秒发送一次
	unsigned long currentTime = millis();
	if (currentTime - lastSendTime >= sendInterval) {
		sendGPSData();  // 发送固定的假坐标
		lastSendTime = currentTime;
	}
	
	delay(100); // 小延时避免过于频繁的循环
}