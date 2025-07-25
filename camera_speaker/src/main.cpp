#define CAMERA_MODEL_ESP32S3_EYE
#define BUTTON_PIN    0
#define I2S_DOUT      48
#define I2S_BCLK      47
#define I2S_LRC       3

#include "esp_camera.h"
#include "camera_pins.h"
#include "ws2812.h"
#include "Base64.h"
#include "sd_read_write.h"
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include <freertos/semphr.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "Audio.h"
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <esp_now.h>
#include <string.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <esp_wifi.h>      // for esp_wifi_set_channel()
#include <esp_now.h>
#include <Wire.h>

HardwareSerial gy39Serial(2);
uint8_t slaveMac[6] = { 0xE8, 0x68, 0xE7, 0x31, 0x0E, 0x50 };
const char* ssid = "cole";
const char* password = "abcabcabc";
AsyncWebServer server(80);
AsyncWebSocket wss("/ws");
int currentMode = 2;
Audio audio;

unsigned long lastQueryTime = 0;
unsigned long lastTempQueryTime = 0;
const unsigned long queryInterval = 3000;
const unsigned long tempQueryInterval = 1500;

struct {
  float latitude;
  float longitude;
  bool hasValidLocation;
  float temperature;
  float humidity;
} sensorData;

typedef struct __attribute__((packed)) {
  bool  isSet;
  char  act[16];
  float val;
} Packet;

Packet txPkt;
const float C_nom_Ah = 2.0f;
float SoC      = 100.0f;
float Qused_Ah = 0.0f;
float I_prev   = NAN;
unsigned long t_prev_ms = 0;


// loop timing 
const unsigned long LOOP_MS = 1000;

const int   PIN_VLOGIC = 4;   
const int   PIN_VMOTOR = 5;  
const int   PIN_VBAT = 6;  
const float VREF       = 3.3f; 
const float DIV_RATIO = 0.25f;    
const float ADC_MAX    = 4095.0f;

inline float readVolts(int pin)
{
  return analogRead(pin) / ADC_MAX * VREF;
}

const int   N_TAB = 26;
const float V_TAB[N_TAB] = {
  14.880f, 14.340f, 13.800f, 13.500f, 13.200f, 13.104f,
  13.020f, 12.960f, 12.900f, 12.840f, 12.780f, 12.744f,
  12.720f, 12.684f, 12.660f, 12.660f, 12.660f, 12.504f,
  12.360f, 12.180f, 12.000f, 11.640f, 11.280f, 10.860f,
  10.440f,  9.600f
};

const float SOC_TAB[N_TAB] = {
 100.0f,  96.0f,  92.0f,  88.0f,  84.0f,  80.0f,
  76.0f,  72.0f,  68.0f,  64.0f,  60.0f,  56.0f,
  52.0f,  48.0f,  44.0f,  40.0f,  36.0f,  32.0f,
  28.0f,  24.0f,  20.0f,  16.0f,  12.0f,   8.0f,
   4.0f,   0.0f
};

const float R_INT_PACK = 0.12f;


float lookupSoC(float Vocv) {
  if (Vocv >= V_TAB[0])          return 100.0f;
  if (Vocv <= V_TAB[N_TAB-1])    return   0.0f;
  int i = 0;
  // V_TAB is sorted descending
  while (Vocv < V_TAB[i+1]) ++i;
  float frac = (V_TAB[i] - Vocv) / (V_TAB[i] - V_TAB[i+1]);
  return SOC_TAB[i] + frac * (SOC_TAB[i+1] - SOC_TAB[i]);
}

void onMasterSend(const uint8_t*, esp_now_send_status_t status) {
  Serial.printf("ESP-NOW send: %s\n", status==ESP_NOW_SEND_SUCCESS ? "OK":"FAIL");
}

void onMasterRecv(const uint8_t*, const uint8_t* d, int len){
  Serial.printf("Received");
}

void handleJoystick(const String& cmd) {
  memset(&txPkt, 0, sizeof(txPkt));
  txPkt.isSet = false;
  strncpy(txPkt.act, cmd.c_str(), sizeof(txPkt.act)-1);
  txPkt.val = 0;
  esp_err_t result = esp_now_send(slaveMac, (const uint8_t *)&txPkt, sizeof(txPkt));
  if (result != ESP_OK) {
    Serial.printf("esp_now_send error: %d\n", result);
  };
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t * data, size_t len) {
  if (type == WS_EVT_DATA) {
    String cmd;
    for (size_t i = 0; i < len; i++) {
      cmd += char(data[i]);
    }
    cmd.trim();
    if (cmd.length() > 0) {
      handleJoystick(cmd);
    }
  }
}

void initGY39() {
  gy39Serial.begin(9600, SERIAL_8N1, 20, 21);  // DR, CR
  delay(500);
  uint8_t configCmd1[] = {0xA5, 0x83, 0x28};
  gy39Serial.write(configCmd1, 3);
  delay(300);
  Serial.println("GY-39 初始化完成");
}

void updateSensorDataFromGY39() {
  uint8_t queryTempCmd[] = {0xA5, 0x52, 0xF7};
  gy39Serial.write(queryTempCmd, 3);
  delay(300);

  uint8_t buffer[20];
  int len = 0;
  unsigned long startTime = millis();
  while (millis() - startTime < 500 && len < 20) {
    if (gy39Serial.available()) {
      buffer[len++] = gy39Serial.read();
    }
  }

  if (len >= 14 && buffer[0] == 0x5A && buffer[1] == 0x5A && buffer[2] == 0x45) {
    uint16_t temp = (buffer[4] << 8) | buffer[5];
    float temperature = temp / 100.0;

    uint16_t hum = (buffer[10] << 8) | buffer[11];
    float humidity = hum / 100.0;

    sensorData.temperature = temperature;
    sensorData.humidity = humidity;
    sensorData.hasValidLocation = true;

  } else {
    Serial.print("GY-39 无效数据: ");
    for (int i = 0; i < len; i++) {
        if (buffer[i] < 0x10) Serial.print("0");
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
  }
}

/*
int cameraSetup(void) {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 4;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  // for larger pre-allocated frame buffer.
  if(psramFound()){
    config.jpeg_quality = 4;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    // Limit the frame size when PSRAM is not available
    config.frame_size = FRAMESIZE_240X240;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return 0;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  s->set_vflip(s, 1); // flip it back
  s->set_brightness(s, 1); // up the brightness just a bit
  s->set_saturation(s, 0); // lower the saturation

  Serial.println("Camera configuration complete!");
  return 1;
}

String urlencode(String str)
{
    String encodedString="";
    char c;
    char code0;
    char code1;
    char code2;
    for (int i =0; i < str.length(); i++){
      c=str.charAt(i);
      if (c == ' '){
        encodedString+= '+';
      } else if (isalnum(c)){
        encodedString+=c;
      } else{
        code1=(c & 0xf)+'0';
        if ((c & 0xf) >9){
            code1=(c & 0xf) - 10 + 'A';
        }
        c=(c>>4)&0xf;
        code0=c+'0';
        if (c > 9){
            code0=c - 10 + 'A';
        }
        code2='\0';
        encodedString+='%';
        encodedString+=code0;
        encodedString+=code1;
        //encodedString+=code2;
      }
      yield();
    }
    return encodedString;
}

void camera_task(void *parameter) {
  while (1) {
    if(digitalRead(BUTTON_PIN)==LOW){
      delay(20);
      if(digitalRead(BUTTON_PIN)==LOW){
        ws2812SetColor(3);
        while(digitalRead(BUTTON_PIN)==LOW);

        camera_fb_t * fb = NULL;
        fb = esp_camera_fb_get();
        if(!fb) {
          Serial.println("Camera capture failed");
        } else {
          char *input = (char *)fb->buf;
          char output[base64_enc_len(3)];
          String imageFile = "";
          for (int i=0;i<fb->len;i++) {
            base64_encode(output, (input++), 3);
            if (i%3==0) imageFile += urlencode(String(output));
          }
          WiFiClient client;
          HTTPClient http;
          http.begin(client, server);
          http.addHeader("Content-Type", "application/x-www-form-urlencoded");
          int httpResponseCode = http.POST("image="+imageFile+"&id=1");
          vTaskDelay(15000);
          audio.connecttohost("http://192.168.10.142:5000/download");
          http.end();
        }

        esp_camera_fb_return(fb);
        ws2812SetColor(2);
      }
    }
  }
}
*/

struct Result {
  float P_total;
  float SoC;
} latestBattery;

SemaphoreHandle_t resMutex;

void batteryTask(void *pv) {
  const TickType_t period = pdMS_TO_TICKS(1000);
  for (;;) {
    // ---- do the work you want to run forever ----
    unsigned long now_ms = millis();
    float dt_s  = (now_ms - t_prev_ms) / 1000.0f;
    t_prev_ms   = now_ms;
    float dt_h  = dt_s / 3600.0f;

    /* ---- sensor voltages ---- */
    float Vlogic = readVolts(PIN_VLOGIC);
    float Vmotor = readVolts(PIN_VMOTOR);

    /* ---- currents ---- */
    float Ilogic = ((Vlogic - 0.606f) / 510.0f) / 0.01f;
    float Imotor = ((Vmotor - 1.27f) / 100.0f) / 0.01f;
    Ilogic = max(0.0f, Ilogic);
    Imotor = max(0.0f, Imotor);
    float I_now = Ilogic + Imotor;

    /* ---- trapezoidal charge integration ---- */
    if (!isnan(I_prev)) {
      Qused_Ah += 0.5f * (I_prev + I_now) * dt_h;
    }
    I_prev = I_now;

    SoC = constrain(100.0f - (Qused_Ah / C_nom_Ah * 100.0f), 0.0f, 100.0f);
    Serial.print("SoC: ");
    Serial.print(SoC, 1);
    Serial.println(" %");

    /* ---- total power ---- */
    float P_total = Vlogic * Ilogic + Vmotor * Imotor;

    /* ---- print currents to terminal ---- */
    Serial.print("Ilogic: ");
    Serial.print(Ilogic, 5);
    Serial.print(" A    Imotor: ");
    Serial.print(Imotor, 5);
    Serial.println(" V");
    Serial.print("Vlogic: ");
    Serial.print(Vlogic, 3);
    Serial.print(" V    Vmotor: ");
    Serial.print(Vmotor, 3);
    Serial.println(" A");
    // ---- copy into the shared structure ----
    xSemaphoreTake(resMutex, portMAX_DELAY);
    latestBattery.P_total = P_total;
    latestBattery.SoC     = SoC;
    xSemaphoreGive(resMutex);

    vTaskDelay(max(10UL, LOOP_MS - (millis() - now_ms)));       // ALWAYS yield → keeps other tasks happy
  }
}

void audio_task(void *parameter) {
  while (1) {
    audio.loop();
  }
}

void track() {
}

void weather() {
  updateSensorDataFromGY39();
  sensorData.latitude = 114.36;
  sensorData.longitude = 30.54;

}

void remote() {
}

void chat() {
  while (1) {
    audio.loop();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  uint8_t chan = WiFi.channel();
  Serial.printf("Using channel %d for ESP-NOW\n", chan);

  sdmmcInit();
  initGY39();
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error init ESP-NOW");
    while(1) delay(1);
  }
  
  esp_now_register_send_cb(onMasterSend);
  esp_now_register_recv_cb(onMasterRecv);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, slaveMac, 6);
  peerInfo.channel = chan;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
  }
  
  wss.onEvent(onWsEvent);
  server.addHandler(&wss);
  server.on("/set_mode", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("mode", true)) {
      String modeStr = request->getParam("mode", true)->value();
      Serial.println(modeStr);
      if (modeStr == "track") currentMode = 1;
      else if (modeStr == "weather") currentMode = 2;
      else if (modeStr == "remote") currentMode = 3;
      else if (modeStr == "chat") currentMode = 4;
      else {
        request->send(400, "text/plain", "Invalid mode");
        return;
      }
      Serial.println("[HTTP] Mode changed to: " + modeStr);
      request->send(200, "text/plain", "Mode set to " + modeStr);
    } else {
      request->send(400, "text/plain", "Missing 'mode' parameter");
    }
  });
  server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "File received");
  }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  static File uploadFile;
    if (index == 0) {
      String path = "/" + filename;
      Serial.printf("Upload Start: %s\n", path.c_str());
      uploadFile = SD_MMC.open(path, FILE_WRITE);
      if (!uploadFile) {
        Serial.println("Failed to open file for writing");
        return;
      }
    }
    uploadFile.write(data, len);
    if (final) {
      Serial.printf("Upload Complete: %s (%u bytes)\n", filename.c_str(), index + len);
      uploadFile.close();
    }
  });
  server.on("/weather", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<200> doc;
    weather();
    doc["temperature"] = sensorData.temperature;
    doc["humidity"] = sensorData.humidity;
    if (sensorData.hasValidLocation) {
      doc["lat"] = sensorData.latitude;
      doc["lng"] = sensorData.longitude;
    } else {
      doc["lat"] = nullptr;
      doc["lng"] = nullptr;
    }
    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
  });
  server.on("/battery", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<128> doc;

    xSemaphoreTake(resMutex, portMAX_DELAY);
    doc["power"] = latestBattery.P_total;
    doc["percent"]    = latestBattery.SoC;
    xSemaphoreGive(resMutex);

    String payload;
    serializeJson(doc, payload);
    request->send(200, "application/json", payload);
  });
  server.begin();

  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(10); // default 0...21
  //audio.connecttoFS(SD_MMC, "/response.mp3");
  //audio.connecttohost("longfei.store:8000/api/cart/");

  float Vadc      = readVolts(PIN_VBAT);      
  float Vpack     = Vadc / DIV_RATIO;         

  float SoC_start = lookupSoC(Vpack);  
  Qused_Ah        = C_nom_Ah * (1.0f - SoC_start/100.0f);  
  I_prev          = 0.0f;
  t_prev_ms       = millis();
  Serial.print("bat: ");
  Serial.print(Vpack, 1);
  
  Serial.print("Seeded SoC at 13 V = ");
  Serial.print(SoC_start,1);
  Serial.println(" %");
  resMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(batteryTask, "battery", 4096, nullptr, 3, nullptr, 0);     
  /*
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  ws2812Init();
  if (cameraSetup()==1) {
    ws2812SetColor(2);
  } else {
    ws2812SetColor(1);
  }
  xTaskCreatePinnedToCore(movement_task, "movement", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(camera_task, "camera", 10000, NULL, 3, NULL, 1);
  */
  //xTaskCreatePinnedToCore(audio_task, "audio", 10000, NULL, 3, NULL, 1);
}

void loop() {
  /*
  switch (currentMode) {
    case 1:
      track();
      break;
    case 2:
      weather();
      break;
    case 3:
      remote();
      break;
    case 4:
      chat();
      break;
    default:
      weather();
  }
  */
  //audio.loop();
}