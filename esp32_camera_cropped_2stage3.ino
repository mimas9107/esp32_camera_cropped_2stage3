/* Includes */
#include <vector>
#include <datasetchange_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "edge-impulse-sdk/classifier/ei_classifier_types.h"
#include "esp_camera.h"
#include <Adafruit_NeoPixel.h>

#include <Arduino.h>
#include <WiFi.h>
#include "time.h"
#include "esp_sleep.h"
#include <ESP_Google_Sheet_Client.h>
#include "credential.h"

#define LEDPIN   2  // Busy LED    for ESP32 DOIT DevKitC V1, and ESP32 CAM
#define INITLED  13 // Intial LED      for ESP32 DOIT DevKitC V1, and ESP32 CAM
#define IDLELED  14 // IDLE standy LED for ESP32 DOIT DevKitC V1, and ESP32 CAM

#define PIN        12
#define NUMPIXELS 16
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 30

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS  1024
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS  768
#define EI_CAMERA_FRAME_BYTE_SIZE        3
#define CLASS_CONFIDENCE                 0.50f
#define DETECT_TIME_THRESHOLD            4000  // 偵測時間上限 (10秒)
#define MAX_DETECTION_COUNT              1      // 偵測次數上限
#define GET_ROUND                        2     // 取第2回合的資料
#define MAX_ROUND                        3
#define MAX_NTP_RETRY                    8

// #define SUEDOINFERENCE 1 // ※設定模擬開關 1關推論改用亂數產生模擬, 要推論這行要註解起來
// #define SCHEDULE_INTERVAL 600000 // gsheet上傳完之後就 standby 10分鐘
#define SCHEDULE_INTERVAL 60000 // gsheet上傳完之後就 standby 1分鐘
#define UPLOAD_SWITCH 1 // 設定要不要上傳 gsheet, 測試穩定性時可以關掉(註解起來)



bool initialboot=true;   // 判斷是不是剛啟動

static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf;

int gsheet_status=0;

int detect_status = 0;
int cur_grid = 0; // loop()中控制目前藥格
int rounds=1;   // loop()中控制偵測回合 


const int detectTimes[][4]={
  // {6,10,6,25},{7,10,7,25},{11,10,11,25},{12,10,12,25},{17,10,17,25},{18,10,18,25} // 原定義偵測區段
  {15,10,15,25},{15,29,15,40},{15,50,15,59},{16,5,16,15},{16,20,16,30},{16,40,16,50},{17,0,17,10},{17,20,17,30},{17,40,17,50},{17,55,18,10},{18,20,18,30}, // 測試用定義偵測區段
  {18,30,18,40},{18,50,19,0},{19,10,19,20},{19,30,18,40},{19,50,20,0},{20,10,20,20},{20,30,20,40},{20,40,20,50},{21,0,21,10},
  {21,30,21,40},{21,50,22,0},{22,10,22,20},{22,30,22,40},{22,50,23,0},{23,10,23,20},{23,30,23,40},{23,40,23,50}
};
const int NUM_DETECT_TIMES = sizeof(detectTimes) / sizeof(detectTimes[0]);

struct points {
    int x;
    int y;
}; 

std::vector<points> ROI = {
    {124,477},{246,498},{386,500},{525,507},{661,501},{781,489},
    {120,333},{248,335},{381,336},{526,338},{662,336},{790,337},
    {124,178},{249,168},{382,170},{525,160},{663,166},{790,177}
};

class pillbox_manager {
private:
    std::vector<std::vector<ei_impulse_result_bounding_box_t>> grid;
public:
    pillbox_manager() {
        grid.resize(18);
        for(int i=0; i<18; i++){
          grid[i]=std::vector<ei_impulse_result_bounding_box_t>();
        }
    }

    void record(const std::vector<ei_impulse_result_bounding_box_t> &detected_pills, int grid_id) {
        if ( grid_id<0 || grid_id >= 18) {
            ei_printf("[pillbox_manager] Failed to put data in record!\n");
            return;
        }
        grid[grid_id] = detected_pills; // 直接覆蓋最新結果
        ei_printf("[pillbox_manager]Recorded grid[%d]: %d items: ", grid_id, (int)detected_pills.size());
        
        if(detected_pills.empty()){
          // ei_impulse_result_bounding_box_t tmp={};
          // grid[grid_id]=&tmp;
          ei_printf("[pillbox_manager] No pills detected in this grid\n");
        }else{
          for(int i=0; i<detected_pills.size();i++){
            ei_printf("{%s, %f}, ", detected_pills[i].label, detected_pills[i].value);
          }
          ei_printf("\n");
        }        
        
    }
    const std::vector<std::vector<ei_impulse_result_bounding_box_t>>& getGrid() const {
        return grid;
    }


    void print_grid() {
      ei_printf("=====================================================================================\n");
        for (size_t i = 0; i < grid.size(); i++) {
            // ei_printf("grid[%d]: %d items\n", (int)i, (int)grid[i].size());
            if(grid[i].empty()){
              ei_printf("%d. [pillbox_manager] No pills detected, ( 0 items)\n",i);
            }
            else{
              ei_printf("%d. %d items: ",i,(int)grid[i].size());
              for (const auto &item : grid[i]) {
                ei_printf("  %s, value:%f, x: %d, y: %d\n", item.label, item.value, item.x, item.y);
              }
              ei_printf("\n");
            }            
        }
      ei_printf("=====================================================================================\n");
    }
};


//---------------------------------------------------------------------------------------------------------
//function forward declare:
void tokenStatusCallback(TokenInfo info);
void uploadToGoogleSheet(pillbox_manager& manager);
void printLocalTime();
int connectToWiFi();
void initPixels();
void offPixels();
void onPixels();
bool isDetectTime();
unsigned long getNextScheduleInterval();

//---------------------------------------------------------------------------------------------------------
bool isDetectTime() {
  struct tm timeinfo;
  
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return false;
  }
  
  int currentHour = timeinfo.tm_hour;
  int currentMinute = timeinfo.tm_min;
  int currentTime=currentHour*60+currentMinute;
  // Serial.printf("[isDetectTime] currentTime=%d\n",currentTime); // 確認時間.

  for (int i = 0; i < NUM_DETECT_TIMES; i++) {
    int startTime=detectTimes[i][0]*60+detectTimes[i][1];
    int endTime=detectTimes[i][2]*60+detectTimes[i][3];
    if (currentTime>=startTime && currentTime<endTime) {
      Serial.printf("[isDetectTime] currentHour=%02d, currentMinute=%02d \n",currentHour,currentMinute);
      return true;
      
    }
  }
  return false;
}

unsigned long getNextScheduleInterval() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return 60000; // 若無法取得時間, 預設 60 秒後再試
  }

  int currentHour = timeinfo.tm_hour;
  int currentMinute = timeinfo.tm_min;
  int currentTime = currentHour * 60 + currentMinute;
  
  int nextTime = 24 * 60; // 預設為隔天第一個排程

  for (int i = 0; i < NUM_DETECT_TIMES; i++) {
    int startTime = detectTimes[i][0] * 60 + detectTimes[i][1];
    if (startTime > currentTime) {
      nextTime = startTime;
      break;
    }
  }

  // 若已是最後一個排程，設定為隔天第一個時間
  if (nextTime == 24 * 60) {
    nextTime = detectTimes[0][0] * 60 + detectTimes[0][1];
  }

  int sleepMinutes = nextTime - currentTime;
  if (sleepMinutes < 0) sleepMinutes += 24 * 60; // 跨天處理

  Serial.printf("Next schedule in %d minutes\n", sleepMinutes);
  return sleepMinutes * 60 * 1000; // 轉成毫秒
}


void initPixels(){
  pixels.begin();
  
  offPixels();
  onPixels();
  Serial.println("Pixels initialiezed!");
}
void offPixels(){
  Serial.println("Turn off light");
  pixels.clear();
  pixels.show();
}
void onPixels(){
  Serial.println("Turn on light");
  for(int i = 0; i < 16; i++) {
        pixels.setPixelColor(i, pixels.Color(68,96,96));
        pixels.show();
        delay(DELAYVAL);
  }
}


void uploadToGoogleSheet(pillbox_manager& manager){
  if(!GSheet.ready()){
    Serial.println("Google Sheet client not ready!");
    return;
  }

  FirebaseJson valueRange;
  FirebaseJson response;

  valueRange.add("range", "Sheet1!A1:O20"); // 目前支援9顆藥 C,D,E,...,J,K欄; 
  valueRange.add("majorDimension","ROWS");
  // 設定標題列 +時間戳記
  valueRange.set("values/[0]/[0]","compartment"); // A1
  valueRange.set("values/[0]/[1]","count"); // B1
  valueRange.set("values/[0]/[2]","medication"); // C1
  // valueRange.set("values/[0]/[3]",timestr); // D1
  for(int i=1; i<8; i++){
    valueRange.set("values/[0]/["+String(i+2)+"]","medication"+String(i)); // 變更成 3:D1, 4:E1, 5:F1, 6:G1, 7:H1, 8:I1, 9:J1  
  }
  // for(int i=1; i<4; i++){ 
  //   valueRange.set("values/[0]/["+String(i+9)+"]","day"+String(i)+"complete"); // 10: K1 day1complete, 11: L1 day2complete, M: day3complete
  // }
  valueRange.set("values/[0]/[10]","Timestamp");
  valueRange.set("values/[0]/[11]","Date");
  valueRange.set("values/[0]/[12]","Taken");

  // 上傳前先清空表格，因前次藥品名有可能擴展至多欄位，
  Serial.println("\n Clear spreadsheet values in range...");
  //     看情況用 .batchClear() ie. "Sheet1!A5:A8,Sheet1!B1:C10" 只清掉局部區域.
  bool success = GSheet.values.clear(&response, GOOGLE_SHEET_ID, "Sheet1!A2:M20");
  if (success) {
      Serial.println("Data clear successfully!");
      response.toString(Serial, true);
      Serial.println();
  }
  // 2025/03/16 依照gpt建議加入延遲時間測試.

  ei_sleep(3000);
  // 取得時間準備寫入時間戳記
  char timeStr[20];
  char dateStr[15];
  struct tm timeinfo;
  
  const auto& grid=manager.getGrid();
  
  for(size_t i=0; i<18; i++){

    if (getLocalTime(&timeinfo)) {
      strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo); // 例如: 2025-03-17 06:15:32
      strftime(dateStr, sizeof(dateStr), "%Y-%m-%d", &timeinfo);          // 例如: 2025-03-17
    } else {
      strcpy(timeStr, "N/A");
      strcpy(dateStr, "N/A");
    }
    valueRange.set("values/["+String(i+1)+"]/[0]",String(i+1)); // Grid 編號 (A列)

    if(!grid[i].empty()){
      // const auto &item=grid[i][0];
      // 關鍵修改：即使沒有偵測到藥丸，也要上傳該格資訊 
      
      int pill_count=grid[i].size(); //偵測到的藥丸數量
      String taken=(pill_count==0)?"Yes":"No"; // 沒有藥丸代表已經服用
      
      valueRange.set("values/["+String(i+1)+"]/[1]",String(pill_count)); // 藥丸數量 (B欄)
      for(size_t j=0; j<grid[i].size();j++){
        valueRange.set("values/["+String(i+1)+"]/["+String(j+2)+"]",grid[i][j].label); // 標籤 (C欄,D,E,...)
      }
      //如果藥物數量少於8個 補上空白值，防止資料錯位
      for(size_t j=grid[i].size();j<8;j++){
        valueRange.set("values/["+String(i+1)+"]/["+String(j+2)+"]","");
      }
      // 設定 Timestamp, Date, Taken (K欄, L欄, M欄) 數據:
      valueRange.set("values/[" + String(i + 1) + "]/[10]", timeStr);
      valueRange.set("values/[" + String(i + 1) + "]/[11]", dateStr);
      valueRange.set("values/[" + String(i + 1) + "]/[12]", taken);

    }else{
      // 如果該格沒有藥丸，上傳空值或預設值
      valueRange.set("values/["+String(i+1)+"]/[1]","0"); // 藥丸數量 (B欄)
      valueRange.set("values/["+String(i+1)+"]/[2]","N/A"); // 藥丸標籤="N/A" (C欄)
      //如果藥物數量少於8個 補上空白值，防止資料錯位
      for(size_t j=grid[i].size();j<8;j++){
        valueRange.set("values/["+String(i+1)+"]/["+String(j+2)+"]","");
      }
      valueRange.set("values/[" + String(i + 1) + "]/[10]", timeStr);
      valueRange.set("values/[" + String(i + 1) + "]/[11]", dateStr);
      valueRange.set("values/[" + String(i + 1) + "]/[12]", "Yes");

    }
    ei_sleep(300);
  }
  
  int retryUpdate=1;
  int retry_i=0;
  while(retry_i<3 && retryUpdate ==1){
    bool success = GSheet.values.update(&response, GOOGLE_SHEET_ID, "Sheet1!A1:O20", &valueRange);
    if (success) {
      Serial.println("Data uploaded successfully!");
      retryUpdate=0;
      retry_i=0;
      // return;
      break;
    } else {
      Serial.println("Failed to upload data.");
      Serial.println(GSheet.errorReason());
      ei_printf("Retry upload data %d times\n",retry_i+1);
      retry_i++;
      retryUpdate=1;
      ei_sleep(3000);
    }
  }
}
void tokenStatusCallback(TokenInfo info) {
  if (info.status == token_status_error) {
      Serial.printf("Token error: %s\n", GSheet.getTokenError(info).c_str());
  }
}

// GoogleSheetClient GSheet;

#if defined(SUEDOINFERENCE) // 跑模擬推論
  #include <stdlib.h>
  #include <time.h>
  // void simulated_detection(std::vector<ei_impulse_result_bounding_box_t>);
  
  void simulated_detection(std::vector<ei_impulse_result_bounding_box_t> &detected_pills){

    detected_pills.clear(); // 清空舊的資料
    delay(1000);
    srand((unsigned) time(NULL)); // 設定隨機種子
    
    int num_detections=rand()%4; // 為產生隨機 0~3個結果,
    // if(num_detections==0){
    //   // {"",0,0,0,0,1.0}
    //   detected_pills.push_back({"",0,0,0,0,1.0});
    //   return;
    // }

    for(int i=0;i<num_detections; i++){
      static char label_buffer[16]; // 緩衝區來放標籤,
      delay(1000);
      srand((unsigned) time(NULL)); // 重設定隨機種子
      
      int randnum=rand()%6;
      ei_impulse_result_bounding_box_t simulated_box;
      snprintf(label_buffer, sizeof(label_buffer), "pill_%d", randnum); // 隨機產生標籤
      simulated_box.label=label_buffer;
      simulated_box.value=(float)(rand()%50+50)/100.0f; // 隨機產生信心度
      simulated_box.x=ROI[cur_grid].x+(rand()%20-10); //隨機產生位置x
      simulated_box.y=ROI[cur_grid].y+(rand()%20-10); //隨機產生位置y
      simulated_box.width=rand()%40+10; //隨機產生寬度w
      simulated_box.height=rand()%40+10; //隨機產生高度h
      detected_pills.push_back(simulated_box);
    }
  }
#else // 跑EI推論
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_XGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 8, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;

void run_inference();

#endif


// 設定我的偵測結果管理員物件:
pillbox_manager pillbox_mgr;

//網路校正時間
const char* ntpServer="pool.ntp.org";
const long gmtOffset_sec=8*60*60;
const int daylightOffset_sec=0;
void printLocalTime(){
  struct tm timeinfo;
  int retry_i=0;
  int Max_retry=5;
  while(retry_i<MAX_NTP_RETRY){
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
      retry_i++;
      delay(3000);
    }
    else
    {
      break;
    }
  }
  Serial.println(&timeinfo,"%A, %B %d %Y %H:%M:%S");
  Serial.println("Time variables");
  
  char timeHour[3];
  strftime(timeHour,3,"%H",&timeinfo);
  Serial.println(timeHour);

  char timeWeekDay[10];
  strftime(timeWeekDay,10,"%A",&timeinfo);
  Serial.println(timeWeekDay);
  Serial.println();
  
}


int connectToWiFi() {
  const int wifiCount = sizeof(wifiNetworks) / sizeof(wifiNetworks[0]);
  Serial.println("嘗試連接 Wi-Fi 熱點...");

  WiFi.setAutoReconnect(true);

  for (int i = 0; i < wifiCount; i++) {
    const char* ssid = wifiNetworks[i][0];
    const char* password = wifiNetworks[i][1];

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("已經成功連接，不需要再次呼叫 WiFi.begin()。");
      break;
    }

    Serial.printf("嘗試連接到 SSID: %s\n", ssid);
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
      delay(1000);  // 增加等待時間，避免過於頻繁的 Wi-Fi 嘗試
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\n成功連接到 SSID: %s\n", ssid);
      Serial.printf("IP 地址: %s\n", WiFi.localIP().toString().c_str());

      delay(1000);
      Serial.printf("NTP calibration");
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      printLocalTime();

      return WiFi.status();
    } else {
      Serial.printf("\n無法連接到 SSID: %s,嘗試下一個...\n", ssid);
      WiFi.disconnect(true);  // 清理之前的連線嘗試
      delay(1000);  // 短暫延遲，給系統時間進行重置
    }
  }

  Serial.println("所有 Wi-Fi 熱點均無法連接，請檢查設定或環境。");
  return WiFi.status();
}



void setup() {
    pinMode(LEDPIN, OUTPUT);  // 設定忙碌燈號輸出
    pinMode(INITLED, OUTPUT); // 設定初始化燈號輸出
    pinMode(IDLELED, OUTPUT); // 設定待機燈號輸出

    digitalWrite(INITLED, HIGH); //初始化開始, 開初始化燈號
    Serial.begin(115200);
        
    initialboot=true;
    
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    GSheet.printf("ESP Google Sheet Client v%s\n\n", ESP_GOOGLE_SHEET_CLIENT_VERSION);

    gsheet_status=connectToWiFi();

  // 有連上線再將 Google Sheet物件建立
  if(gsheet_status == WL_CONNECTED){
    GSheet.setTokenCallback(tokenStatusCallback);
    GSheet.setPrerefreshSeconds(14 * 60);
    GSheet.begin(CLIENT_EMAIL, PROJECT_ID, PRIVATE_KEY);
  }
#ifndef SUEDOINFERENCE
  if (!ei_camera_init()) {
      ei_printf("Failed to initialize Camera!\n");
  } else {
      ei_printf("Camera initialized\n");
  }
#endif
  initPixels();
  onPixels();

  // ei_printf("Starting continuous inference in 2 seconds...\n");
  ei_printf("\n[setup()] All functions have initialized. Ready to go!\n");
  delay(2000);
    
  digitalWrite(INITLED, LOW); // 初始化完成, 關閉初始化燈號
}


void loop()
{
  if (!isDetectTime()) {
    // ei_sleep(1000); // 非偵測時間，進入待機狀態
    unsigned long sleepTime=getNextScheduleInterval();
    Serial.printf("Sleeping for %lu ms until next schedule...\n",sleepTime);
    digitalWrite(IDLELED, HIGH); // 進待機模式, 開待機燈號
    ei_sleep(sleepTime);
    return;
  }else
  {
    Serial.println("Starting detection...");
    digitalWrite(IDLELED, LOW); // 偵測模式, 關待機燈號
  }

  if (ei_sleep(5) != EI_IMPULSE_OK) {
    return;
  }
  // 若到第3回合, 暫停SCHEDULE_INTERVAL秒 印出物件 pillbox_mgr紀錄的結果，並上傳 Google sheet
  if(rounds>MAX_ROUND){
    pillbox_mgr.print_grid();
#if defined(UPLOAD_SWITCH) //設定要不要上傳 gsheet, 測試時這裡會不編譯
      uploadToGoogleSheet(pillbox_mgr);
#endif
    Serial.println("Stand by ..................");
    offPixels(); // 關燈條
    digitalWrite(LEDPIN, LOW);
    // ei_sleep(SCHEDULE_INTERVAL); // 原始固定間隔時間邏輯
    // 睡到下一個排程時間
    unsigned long sleepTime=getNextScheduleInterval();
    Serial.printf("Sleeping for %lu ms until next schedule...\n",sleepTime);
    digitalWrite(IDLELED,HIGH);
    ei_sleep(sleepTime);

    onPixels(); // 開燈條
    digitalWrite(LEDPIN, HIGH);
    rounds=1;
  }
    
  static unsigned long start_time = 0;
  static int detect_count = 0;
  static std::vector<ei_impulse_result_bounding_box_t> detected_pills;
      
  if (cur_grid >= 18) {
      ei_printf("All grids processed. Restarting...\n");
      cur_grid = 0;
      rounds++;
  }    

  // 設定偵測時間
  if (detect_count == 0) {
      start_time = millis();
      detected_pills.clear();
  }
    //原來判斷目前格子的邏輯:
    //if (millis() - start_time > (cur_grid == 0? DETECT_TIME_THRESHOLD+16000: DETECT_TIME_THRESHOLD) || detect_count >= MAX_DETECTION_COUNT) { 
  if (millis() - start_time > DETECT_TIME_THRESHOLD || detect_count >= MAX_DETECTION_COUNT) {
      // 判斷是否要紀錄該格子的結果
      pillbox_mgr.record(detected_pills, cur_grid);
      cur_grid++;  // 移動到下一個格子
      detect_count = 0; // 重置計數
      return;
  }

#if defined(SUEDOINFERENCE)
  digitalWrite(LEDPIN, HIGH);
  // 不跑推論, 用亂數模擬
  simulated_detection(detected_pills);
  detect_count++;
  digitalWrite(LEDPIN, LOW);
  delay(80);
#else 
  digitalWrite(LEDPIN, HIGH);
  //跑Edge Impulse推論 run_classifier:
  run_inference();
  detect_count++;  // 增加偵測次數
  digitalWrite(LEDPIN, LOW);
  delay(50);
#endif //SUEDOINFERENCE
}


#if defined(SUEDOINFERENCE)
  //模擬
  
#else // 跑推論要用到的函數本體定義:

bool ei_camera_init() {
    if (is_initialised) return true;
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }
    is_initialised = true;
    return true;
}

/*** @brief 擷取影像並裁剪當前 grid 的 ROI  */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }
    
    camera_fb_t *fb = esp_camera_fb_get();

    // 2025/03/16 若一開機攝影機的framebuffer起初幾秒捕捉到的影像不能用, 
    // 所以採取 return->get-return->get->return->get 共3次(約6秒)
    // 用一個布林狀態旗標 initialboot剛開機為 true, 經此程序後 initialboot改為 false.
    if(initialboot && rounds==1){
      int i=0;
      while(i<3){
        esp_camera_fb_return(fb);
        ei_sleep(1000);
        fb = esp_camera_fb_get();
        ei_sleep(1000);
        i++;
      }
      initialboot=false;
    }   
    
    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);
    ei_sleep(250); // 抓影像之後停一下再送給分類器
    if (!converted) {
        ei_printf("Conversion failed\n");
        return false;
    }

    // 取得當前 grid 的 ROI
    int roi_x = ROI[cur_grid].x;
    int roi_y = ROI[cur_grid].y;

    // 裁剪並縮放到模型輸入大小
    int res = ei::image::processing::crop_image_rgb888_packed(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        roi_x, roi_y,
        out_buf,
        img_width,
        img_height
    );

    if (res != 0) {
        ei_printf("Image crop failed\n");
        return false;
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    return 0;
}

// 2025/03/16 獨立出 inference的區塊
void run_inference(){

    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }
  // 擷取影像並處理
    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    if (!snapshot_buf) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    
    if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
        ei_printf("Failed to capture image\n");
        free(snapshot_buf);
        return;
    }
    ei_sleep(500); // 測試看看在抓影像之後停一下再送給分類器
    // 執行分類器
    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("Classifier failed (%d)\n", err);
        free(snapshot_buf);
        return;
    }

    // 處理偵測結果
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_printf("result.bbcount= %d\n",result.bounding_boxes_count);
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        // if(result.bounding_boxes_count==0){
        //   detected_pills.push_back({"",0,0,0,0,1.0});
        // }
        if (bb.value >= CLASS_CONFIDENCE) {
            detected_pills.push_back(bb);
            // detected_pills[i]=bb;
        }
    }    
    free(snapshot_buf);
}

#endif //SUEDOINFERENCE


