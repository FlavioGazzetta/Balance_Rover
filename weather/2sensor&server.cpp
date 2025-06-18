#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>

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
// GY-39: CT<=> esp 21, DR<=> esp 22

// 创建GY-39串口对象
HardwareSerial gy39Serial(1);

// 全局传感器数据变量
struct {
    float temperature;
    float humidity;
    float pressure;
    float light;
    uint16_t altitude;
    bool hasValidData;
} GlobalSensorData;

// 全局服务器响应数据
struct {
    String location;
    float server_temperature;
    float feels_like;
    int server_humidity;
    bool hasValidResponse;
} GlobalServerData;

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

// 显示模式控制
unsigned long lastModeChangeTime = 0;
const unsigned long modeChangeInterval = 5000; // 5秒切换模式
bool displayMode = false; // false=服务器数据模式, true=传感器数据模式

// GY-39传感器定时器
unsigned long lastGY39QueryTime = 0;
unsigned long lastTempQueryTime = 0;
const unsigned long gy39QueryInterval = 3000; // 每3秒查询一次
const unsigned long tempQueryInterval = 1000; // 温湿度查询间隔

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
    }
}

// 检查是否需要使用默认位置
void checkAndSetDefaultLocation() {
    unsigned long currentTime = millis();
    
    // 如果超过设定时间没有GPS更新，或者从未有过有效GPS位置
    if (!GlobalLocation.hasValidLocation || 
        (GlobalLocation.isGPSLocation && (currentTime - lastGPSUpdateTime > gpsTimeoutDuration))) {
        setDefaultLocation();
    }
}

// 简单解析服务器响应JSON (不使用ArduinoJson库)
void parseServerResponse(String response) {
    // 查找各个字段
    int locationStart = response.indexOf("\"location\":\"") + 12;
    int locationEnd = response.indexOf("\"", locationStart);
    if (locationStart > 11 && locationEnd > locationStart) {
        GlobalServerData.location = response.substring(locationStart, locationEnd);
    }
    
    int tempStart = response.indexOf("\"temperature\":") + 14;
    int tempEnd = response.indexOf(",", tempStart);
    if (tempStart > 13 && tempEnd > tempStart) {
        GlobalServerData.server_temperature = response.substring(tempStart, tempEnd).toFloat();
    }
    
    int feelsStart = response.indexOf("\"feels_like\":") + 13;
    int feelsEnd = response.indexOf(",", feelsStart);
    if (feelsStart > 12 && feelsEnd > feelsStart) {
        GlobalServerData.feels_like = response.substring(feelsStart, feelsEnd).toFloat();
    }
    
    int humStart = response.indexOf("\"humidity\":") + 11;
    int humEnd = response.indexOf("}", humStart);
    if (humStart > 10 && humEnd > humStart) {
        String humStr = response.substring(humStart, humEnd);
        humStr.replace("}", ""); // 移除可能的结束括号
        GlobalServerData.server_humidity = humStr.toInt();
    }
    
    GlobalServerData.hasValidResponse = true;
    
    // Debug info: show raw response
    // DebugSerial.println("Raw server response: " + response);
}

void sendGPSData() {
	// 检查并设置默认位置（如果需要）
	checkAndSetDefaultLocation();
	
	if (GlobalLocation.hasValidLocation && WiFi.status() == WL_CONNECTED)
	{
		// 构造POST数据 - 使用全局坐标
		String postData = "id=imperial.ac.uk&lat=" + String(GlobalLocation.latitude, 6) + 
		                  "&lng=" + String(GlobalLocation.longitude, 6);
		
		http.begin(client, server);
		http.addHeader("Content-Type", "application/x-www-form-urlencoded");
		
		int httpResponseCode = http.POST(postData);
		
		if (httpResponseCode > 0) {
			String response = http.getString();
			
			if (httpResponseCode == 200) {
				// 解析服务器响应
				parseServerResponse(response);
				// 成功发送时LED快闪一次
				digitalWrite(L, HIGH);
				delay(100);
				digitalWrite(L, LOW);
			}
		}
		
		http.end();
	}
}

// 解析GY-39光照强度数据
void parseLight(uint8_t* data) {
  uint32_t lux = ((uint32_t)data[4] << 24) | 
                 ((uint32_t)data[5] << 16) | 
                 ((uint32_t)data[6] << 8) | 
                 data[7];
  
  GlobalSensorData.light = lux / 100.0;
  GlobalSensorData.hasValidData = true;
}

// 解析GY-39温湿度气压数据
void parseTempHumPress(uint8_t* data) {
  // 温度
  uint16_t temp = (data[4] << 8) | data[5];
  GlobalSensorData.temperature = temp / 100.0;
  
  // 气压
  uint32_t press = ((uint32_t)data[6] << 24) | 
                   ((uint32_t)data[7] << 16) | 
                   ((uint32_t)data[8] << 8) | 
                   data[9];
  GlobalSensorData.pressure = press / 100.0;
  
  // 湿度
  uint16_t hum = (data[10] << 8) | data[11];
  GlobalSensorData.humidity = hum / 100.0;
  
  // 海拔
  GlobalSensorData.altitude = (data[12] << 8) | data[13];
  
  GlobalSensorData.hasValidData = true;
}

// 查询GY-39传感器数据
void queryGY39Sensors() {
  // 先查询温湿度气压
  uint8_t queryTempCmd[] = {0xA5, 0x52, 0xF7};
  gy39Serial.write(queryTempCmd, 3);
  
  delay(200);
  
  // 再查询光照强度
  uint8_t queryLightCmd[] = {0xA5, 0x51, 0xF6};
  gy39Serial.write(queryLightCmd, 3);
}

// 读取和处理GY-39数据
void readGY39Data() {
  if (gy39Serial.available()) {
    uint8_t buffer[20];
    int len = 0;
    
    delay(50);
    
    while (gy39Serial.available() && len < 20) {
      buffer[len] = gy39Serial.read();
      len++;
    }
    
    // 解析数据
    if (len >= 9 && buffer[0] == 0x5A && buffer[1] == 0x5A) {
      if (buffer[2] == 0x15) {
        // 光照强度数据
        parseLight(buffer);
      } else if (buffer[2] == 0x45) {
        // 温湿度气压数据
        parseTempHumPress(buffer);
      }
    }
  }
}

// Display Mode 1: Server Data
void displayServerData() {
    DebugSerial.println("========== Mode 1: Server Data ==========");
    DebugSerial.print("Location Type: ");
    DebugSerial.println(GlobalLocation.isGPSLocation ? "GPS Coordinates" : "Default Coordinates");
    DebugSerial.print("Coordinates: ");
    DebugSerial.print(GlobalLocation.latitude, 6);
    DebugSerial.print(", ");
    DebugSerial.println(GlobalLocation.longitude, 6);
    
    if (GlobalServerData.hasValidResponse) {
        DebugSerial.print("Address: ");
        DebugSerial.println(GlobalServerData.location);
        DebugSerial.print("Server Temperature: ");
        DebugSerial.print(GlobalServerData.server_temperature);
        DebugSerial.println(" °C");
        DebugSerial.print("Feels Like: ");
        DebugSerial.print(GlobalServerData.feels_like);
        DebugSerial.println(" °C");
        DebugSerial.print("Server Humidity: ");
        DebugSerial.print(GlobalServerData.server_humidity);
        DebugSerial.println(" %");
    } else {
        DebugSerial.println("Waiting for server response...");
    }
    DebugSerial.println("=========================================");
}

// Display Mode 2: Sensor Data
void displaySensorData() {
    DebugSerial.println("========== Mode 2: GY-39 Sensor Data ==========");
    if (GlobalSensorData.hasValidData) {
        DebugSerial.print("Sensor Temperature: ");
        DebugSerial.print(GlobalSensorData.temperature);
        DebugSerial.println(" °C");
        
        DebugSerial.print("Sensor Humidity: ");
        DebugSerial.print(GlobalSensorData.humidity);
        DebugSerial.println(" %");
        
        DebugSerial.print("Light Intensity: ");
        DebugSerial.print(GlobalSensorData.light);
        DebugSerial.println(" lux");
        
        DebugSerial.print("Pressure: ");
        DebugSerial.print(GlobalSensorData.pressure);
        DebugSerial.println(" Pa");
        
        DebugSerial.print("Altitude: ");
        DebugSerial.print(GlobalSensorData.altitude);
        DebugSerial.println(" m");
    } else {
        DebugSerial.println("Waiting for sensor data...");
    }
    DebugSerial.println("===============================================");
}

void printGpsBuffer() {
	if (Save_Data.isParseData)
	{
		Save_Data.isParseData = false;
		
		if(Save_Data.isUsefull)
		{
			// 更新全局位置坐标 (在重置isUsefull之前)
			updateGlobalLocation();
			Save_Data.isUsefull = false;  // 重置标志位
		}
	}
}

void parseGpsBuffer() {
	char *subString;
	char *subStringNext;
	if (Save_Data.isGetData)
	{
		Save_Data.isGetData = false;

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
	
	// 初始化GPS串口
	GpsSerial.begin(115200, SERIAL_8N1, 25, 26); // TX <---> pin 25 of esp32
	// 初始化GY-39串口 (RX=21, TX=22, 波特率=9600)
	gy39Serial.begin(9600, SERIAL_8N1, 21, 22);
	
	DebugSerial.begin(115200);
	DebugSerial.println("ESP32 GPS + GY-39 Sensor System Starting...");

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
	
	// 初始化传感器数据
	GlobalSensorData.temperature = 0.0;
	GlobalSensorData.humidity = 0.0;
	GlobalSensorData.pressure = 0.0;
	GlobalSensorData.light = 0.0;
	GlobalSensorData.altitude = 0;
	GlobalSensorData.hasValidData = false;
	
	// 初始化服务器数据
	GlobalServerData.location = "";
	GlobalServerData.server_temperature = 0.0;
	GlobalServerData.feels_like = 0.0;
	GlobalServerData.server_humidity = 0;
	GlobalServerData.hasValidResponse = false;
	
	lastSendTime = millis();
	lastGPSUpdateTime = 0;
	lastModeChangeTime = millis();
	lastGY39QueryTime = millis();
	lastTempQueryTime = millis();
	
	// 初始查询传感器
	queryGY39Sensors();
	
	DebugSerial.println("Initialization complete, system running...");
	DebugSerial.println("Mode 1: Server Data | Mode 2: GY-39 Sensor Data");
}

void loop()		//主循环
{
	// 检查WiFi连接状态
	if (WiFi.status() != WL_CONNECTED) {
		connectWiFi();
	}
	
	// GPS数据处理
	gpsRead();
	parseGpsBuffer();
	printGpsBuffer();
	
	// GY-39数据处理
	readGY39Data();
	
	unsigned long currentTime = millis();
	
	// 定期发送位置数据到服务器
	if (currentTime - lastSendTime >= sendInterval) {
		sendGPSData();
		lastSendTime = currentTime;
	}
	
	// 定期查询GY-39传感器
	if (currentTime - lastTempQueryTime >= tempQueryInterval) {
		uint8_t queryTempCmd[] = {0xA5, 0x52, 0xF7};
		gy39Serial.write(queryTempCmd, 3);
		lastTempQueryTime = currentTime;
	}
	
	if (currentTime - lastGY39QueryTime >= gy39QueryInterval) {
		uint8_t queryLightCmd[] = {0xA5, 0x51, 0xF6};
		gy39Serial.write(queryLightCmd, 3);
		lastGY39QueryTime = currentTime;
	}
	
	// 模式切换和显示
	if (currentTime - lastModeChangeTime >= modeChangeInterval) {
		if (displayMode == false) {
			// 显示服务器数据
			displayServerData();
			displayMode = true;
		} else {
			// 显示传感器数据
			displaySensorData();
			displayMode = false;
		}
		lastModeChangeTime = currentTime;
	}
	
	delay(100);
}