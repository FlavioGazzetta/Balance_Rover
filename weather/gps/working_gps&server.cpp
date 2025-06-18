#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

//WiFi配置
const char* ssid = "zzc";        // 替换为你的WiFi名称
const char* password = "12345678";  // 替换为你的WiFi密码

//服务器配置
const char* server = "http://3.10.54.50:8000/api/cart/location/";

// 默认位置配置 - 伦敦帝国理工学院南肯辛顿校区
const float DEFAULT_LATITUDE = 51.4988;   // 帝国理工学院纬度
const float DEFAULT_LONGITUDE = -0.1749;  // 帝国理工学院经度

//此处为了兼容其他的多串口Arduino板子
int L = 13; //LED指示灯引脚 

#define GpsSerial Serial2
#define DebugSerial Serial

// ESP32 TX2 = GPIO17, RX2 = GPIO16

// 全局位置坐标变量
struct {
    float latitude;
    float longitude;
    bool hasValidLocation;  // 标记是否有有效位置
    bool isGPSLocation;     // 标记是否为GPS位置（true）还是默认位置（false）
    char lastUpdateTime[12]; // 最后更新时间
} GlobalLocation;

struct {
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
unsigned long lastGPSUpdateTime = 0; // 上次GPS更新时间
const unsigned long gpsTimeoutDuration = 30000; // 30秒无GPS信号后使用默认位置

// WiFi和HTTP客户端
WiFiClient client;
HTTPClient http;

void errorLog(int num) {
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

void clrGpsRxBuffer(void) {
	memset(gpsRxBuffer, 0, gpsRxBufferLength);      //清空
	ii = 0;
}

void connectWiFi() {
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

// 转换NMEA格式坐标到十进制度数
float convertNMEAToDecimal(const char* coord, const char* direction) {
    if (strlen(coord) == 0) return 0.0;
    
    float coordinate = atof(coord);
    
    // 对于纬度: ddmm.mmmm -> dd + mm.mmmm/60
    // 对于经度: dddmm.mmmm -> ddd + mm.mmmm/60
    int degrees;
    float minutes;
    
    if (strlen(coord) > 7) { // 经度格式 dddmm.mmmm
        degrees = (int)(coordinate / 100);
        minutes = coordinate - (degrees * 100);
    } else { // 纬度格式 ddmm.mmmm
        degrees = (int)(coordinate / 100);
        minutes = coordinate - (degrees * 100);
    }
    
    float decimal = degrees + (minutes / 60.0);
    
    // 根据方向调整符号
    if (direction[0] == 'S' || direction[0] == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

// 设置默认位置（帝国理工学院）
void setDefaultLocation() {
    GlobalLocation.latitude = DEFAULT_LATITUDE;
    GlobalLocation.longitude = DEFAULT_LONGITUDE;
    GlobalLocation.hasValidLocation = true;
    GlobalLocation.isGPSLocation = false;
    strcpy(GlobalLocation.lastUpdateTime, "DEFAULT");
    
    DebugSerial.println("=== Using Default Location ===");
    DebugSerial.println("Location: Imperial College London, South Kensington");
    DebugSerial.print("Latitude: ");
    DebugSerial.println(GlobalLocation.latitude, 6);
    DebugSerial.print("Longitude: ");
    DebugSerial.println(GlobalLocation.longitude, 6);
    DebugSerial.println("==============================");
}

// 更新全局位置坐标
void updateGlobalLocation() {
    if (Save_Data.isUsefull) {
        GlobalLocation.latitude = convertNMEAToDecimal(Save_Data.latitude, Save_Data.N_S);
        GlobalLocation.longitude = convertNMEAToDecimal(Save_Data.longitude, Save_Data.E_W);
        GlobalLocation.hasValidLocation = true;
        GlobalLocation.isGPSLocation = true; // 标记为GPS位置
        strcpy(GlobalLocation.lastUpdateTime, Save_Data.UTCTime);
        lastGPSUpdateTime = millis(); // 记录GPS更新时间
        
        DebugSerial.println("=== GPS Location Updated ===");
        DebugSerial.print("Latitude: ");
        DebugSerial.println(GlobalLocation.latitude, 6);
        DebugSerial.print("Longitude: ");
        DebugSerial.println(GlobalLocation.longitude, 6);
        DebugSerial.print("Update Time: ");
        DebugSerial.println(GlobalLocation.lastUpdateTime);
        DebugSerial.println("============================");
    }
}

// 检查是否需要使用默认位置
void checkAndSetDefaultLocation() {
    unsigned long currentTime = millis();
    
    // 如果超过设定时间没有GPS更新，或者从未有过有效GPS位置
    if (!GlobalLocation.hasValidLocation || 
        (GlobalLocation.isGPSLocation && (currentTime - lastGPSUpdateTime > gpsTimeoutDuration))) {
        
        if (!GlobalLocation.hasValidLocation) {
            DebugSerial.println("No GPS signal detected, using default location...");
        } else if (GlobalLocation.isGPSLocation) {
            DebugSerial.println("GPS signal lost for 30 seconds, switching to default location...");
        }
        
        setDefaultLocation();
    }
}

void sendGPSData() {
	// 检查并设置默认位置（如果需要）
	checkAndSetDefaultLocation();
	
	if (GlobalLocation.hasValidLocation && WiFi.status() == WL_CONNECTED)
	{
		// 构造POST数据 - 使用全局坐标
		String postData = "id=imperial.ac.uk&lat=" + String(GlobalLocation.latitude, 6) + 
		                  "&lng=" + String(GlobalLocation.longitude, 6);
		
		DebugSerial.println("Sending location data to server...");
		DebugSerial.print("Location Type: ");
		DebugSerial.println(GlobalLocation.isGPSLocation ? "GPS" : "DEFAULT");
		DebugSerial.println("Data: " + postData);
		
		http.begin(client, server);
		http.addHeader("Content-Type", "application/x-www-form-urlencoded");
		
		int httpResponseCode = http.POST(postData);
		
		if (httpResponseCode > 0) {
			String response = http.getString();
			DebugSerial.println("HTTP Response code: " + String(httpResponseCode));
			DebugSerial.println("Response: " + response);
			
			if (httpResponseCode == 200) {
				DebugSerial.println("Location data sent successfully!");
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
	else if (WiFi.status() != WL_CONNECTED)
	{
		DebugSerial.println("WiFi not connected, skipping send");
	}
}

void printGpsBuffer() {
	if (Save_Data.isParseData)
	{
		Save_Data.isParseData = false;
		
		// DebugSerial.print("Save_Data.UTCTime = ");
		// DebugSerial.println(Save_Data.UTCTime);

		if(Save_Data.isUsefull)
		{
			// DebugSerial.print("Save_Data.latitude = ");
			// DebugSerial.println(Save_Data.latitude);
			// DebugSerial.print("Save_Data.N_S = ");
			// DebugSerial.println(Save_Data.N_S);
			// DebugSerial.print("Save_Data.longitude = ");
			// DebugSerial.println(Save_Data.longitude);
			// DebugSerial.print("Save_Data.E_W = ");
			// DebugSerial.println(Save_Data.E_W);
			
			// 更新全局位置坐标 (在重置isUsefull之前)
			updateGlobalLocation();
			Save_Data.isUsefull = false;  // 重置标志位
		}
		else
		{
			DebugSerial.println("GPS DATA is not useful!");
		}
	}
}

void parseGpsBuffer() {
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
	DebugSerial.println("Starting GPS with Server Upload and Default Location...");

	// 初始化WiFi
	connectWiFi();

	// 初始化GPS数据结构
	Save_Data.isGetData = false;
	Save_Data.isParseData = false;
	Save_Data.isUsefull = false;
	
	// 初始化全局位置
	GlobalLocation.hasValidLocation = false;
	GlobalLocation.isGPSLocation = false;
	GlobalLocation.latitude = 0.0;
	GlobalLocation.longitude = 0.0;
	memset(GlobalLocation.lastUpdateTime, 0, sizeof(GlobalLocation.lastUpdateTime));
	
	lastSendTime = millis();
	lastGPSUpdateTime = 0; // 初始化为0，表示从未收到GPS信号
	
	DebugSerial.println("Setup complete, waiting for GPS data...");
	DebugSerial.println("Will use Imperial College London coordinates if no GPS signal");
}

void loop()		//主循环
{
	// 检查WiFi连接状态
	if (WiFi.status() != WL_CONNECTED) {
		DebugSerial.println("WiFi disconnected, attempting to reconnect...");
		connectWiFi();
	}
	
	gpsRead();	//获取GPS数据
	parseGpsBuffer();//解析GPS数据
	printGpsBuffer();//输出解析后的数据
	
	// 定期发送位置数据（GPS或默认位置）
	unsigned long currentTime = millis();
	if (currentTime - lastSendTime >= sendInterval) {
		sendGPSData();
		lastSendTime = currentTime;
	}
	
	delay(100); // 小延时避免过于频繁的循环
}