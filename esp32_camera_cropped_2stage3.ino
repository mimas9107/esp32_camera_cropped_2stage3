/* Includes */
#include <vector>
#include <datasetchange_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "edge-impulse-sdk/classifier/ei_classifier_types.h"
#include "esp_camera.h"
#include <Adafruit_NeoPixel.h>

#include <Arduino.h>
#include <WiFi.h>
#include <ESP_Google_Sheet_Client.h>

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

// #define SUEDOINFERENCE 1 // ※設定模擬開關 1關推論改用亂數產生模擬, 要推論這行要註解起來

#define PROJECT_ID "my-esp32-proj-449103"
#define CLIENT_EMAIL "myesp32app-service@my-esp32-proj-449103.iam.gserviceaccount.com"
const char PRIVATE_KEY[] PROGMEM = "-----BEGIN PRIVATE KEY-----\n"
"MIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQCv+QRscjBUowxf\n"
"o9zYLuo/uu57i2dyeLziMu1GNpOG/jgF0E9IyQeLzUVnFpAEtVor5J8vsoA5QzTW\n"
"2k/GsOzhEY8vwjvLK7lnQBwYNsP7jlLxebUGXVL54Eg4laJeYuqueaUOQpI8eb3g\n"
"FgWXgvW52BbNMv72oM6ttfaQxizU2jw2m4bqQGaaJhFa9lqCvlEy3++TlsnYr6r7\n"
"cUobjiopzIBwniXIt/4r4YE+7GAo46ybedHQfLa4fKtAsqwBQYNjPyw05hH7U1U0\n"
"qsJgcf/Z92JM8J6WHuBzNCzH/JbE3Swpx+qp1f8qiXD3YkHqqkkIQx+TQlEAc1G9\n"
"34FTeStvAgMBAAECggEANyRwsuzrYTJAp92o0COPaw4QOi2aaoAULEuqF6T1gMk7\n"
"0/KNTbEVCMV3uJCU66zKh1OaYG9uh8McFTGO6yO47uQ6OsyCQS2/6O9pflr2+eaE\n"
"7j9lrwtie4PBDd4x5aLYiHj89GV+/q1lhwlPkkVK7AQfTQjsZWtJqJyLGZ3IIaCF\n"
"+jI15l5VdpgSvAxJWU4zWSD/xe2f4Rp15T17IYJEe73KDH4HiB6FsK6LAiz3wmSF\n"
"X582nA+KiKcD4qe9b3wy0jywCplP/W5FuZZaqtFsvZA7kBrqfQgER8Gi39H7O4AB\n"
"Sz8zH6aI2TSSCXzn/7l/apQAJm9Op0fRCtPmG1sGQQKBgQDkX33KOfDtj48IJ/Bx\n"
"43nuzIFqDPD5RVTO1sA3XJZoRtlj8akZLsuO3GTvKd7uUxGlZ5M/46/Qidx8kuFs\n"
"aXY8hjhoWSNSXY+hMd+ApwtC/TNosfXUnD7Knnc5OpUmUXsIQybPXPKjk9aGp6qA\n"
"eZx9cPhmaaLfzUPUrCzlvSEHLwKBgQDFQrxUYCD0CMHQVf6+A3cPt7WQXZ/qKI7M\n"
"RBAc1kYF1iGxEe0Am4S8W550vd/j0PfCGqTt8VYDcVdO015Tqn89L0SnQj4J28Cv\n"
"Ppv1g3lUWDgSaGHKiA1Bu0JprVFcrJwp+yEMji5WowdEXLuLm3So5Fr2kBDUCeZX\n"
"sgf16okPwQKBgQDFPcBsF+0BkPsNJUmjY7/dQt4HVVQPRxU/a/UqG5qAR6jcjEzr\n"
"RiJjqfC3K6eymSZlgHaKOMGR0HARW968fr2y+o3fehVqvwodQ/DwsdWlLwDmzMUw\n"
"sx38bC82y2Ukaj1j3nO3p2SdaUNgm5FEU0SYhyjTcytMNoH+PhKs/dN32wKBgDWP\n"
"uFlkvK3mjtHF9+SaRkLjTA6GDXzkRFnchU9/MwY0rXNJ/cNzdug6LF1gjSFrxUpU\n"
"N8JQDF3k8aL6q5smKWRwqECOMRA2NiyfYHOanmAzkA4Xf64hoQ/fC9pr8DKsv756\n"
"bO/ez2BA5iE+2MHAlT2iu7xFQ/x/L6dSNO0mIM9BAoGBAIG02ggg/ixpE0f3nhTg\n"
"w5K8YyaGTMuO5x11GoXe4hdw6dBDj2AUas7feTbySkgZgnTWG6K+V8S9U2WAHWGa\n"
"mHBfQPg/CmVkQ/P8PjQw9kQ9E/29TFVQoI+GVtU96R1XGMLC+fFQRr9yFAWodX2I\n"
"fvYADOQcoQ/ZyMqnwNMzSISm\n-----END PRIVATE KEY-----\n";
#define GOOGLE_SHEET_ID "1WiugtRCm7b_a2uu_w6XZCLAgPwsJyT3Z0cIDTx_c8j4"



static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf;

int gsheet_status=0;

int detect_status = 0;
int cur_grid = 0;
int rounds=1;

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
    }

    void record(const std::vector<ei_impulse_result_bounding_box_t> &detected_pills, int grid_id) {
        if (grid_id >= 18) {
            ei_printf("[pillbox_manager] Failed to put data in record!\n");
            return;
        }
        grid[grid_id] = detected_pills; // 直接覆蓋最新結果
        ei_printf("[pillbox_manager]Recorded grid[%d]: %d items: ", grid_id, (int)detected_pills.size());
        for(int i=0; i<detected_pills.size();i++){
          ei_printf("{%s, %f}, ", detected_pills[i].label, detected_pills[i].value);
        }
        ei_printf("\n");
        
    }
    const std::vector<std::vector<ei_impulse_result_bounding_box_t>>& getGrid() const {
        return grid;
    }


    void print_grid() {
      ei_printf("=====================================================================================\n");
        for (size_t i = 0; i < grid.size(); i++) {
            ei_printf("grid[%d]: %d items\n", (int)i, (int)grid[i].size());
            for (const auto &item : grid[i]) {
                ei_printf("  %s, value:%f, x: %d, y: %d\n", item.label, item.value, item.x, item.y);
            }
        }
      ei_printf("=====================================================================================\n");
    }
};
// GoogleSheetClient GSheet;
// void uploadToGoogleSheet(pillbox_manager);
void uploadToGoogleSheet(pillbox_manager& manager){
  if(!GSheet.ready()){
    Serial.println("Google Sheet client not ready!");
    return;
  }
  FirebaseJson valueRange;

  valueRange.add("range", "Sheet1!A2:E19");
  valueRange.add("majorDimension","ROWS");
  const auto& grid=manager.getGrid();
  for(size_t i=0; i<grid.size(); i++){
    if(!grid[i].empty()){
      const auto &item=grid[i][0]; // 假設每個格子只有一個物件
      valueRange.set("values/["+String(i)+"]/[0]",String(i+1)); // Grid 編號 (A列)
      valueRange.set("values/["+String(i)+"]/[1]", item.label); // 標籤 (B列)
      valueRange.set("values/[" + String(i) + "]/[2]", String(item.value, 2)); // 信心度 (C列)
      valueRange.set("values/[" + String(i) + "]/[3]", String(item.x)); // x 座標 (D列)
      valueRange.set("values/[" + String(i) + "]/[4]", String(item.y)); // y 座標 (E列)

    }
  }
  FirebaseJson response;
  bool success = GSheet.values.update(&response, GOOGLE_SHEET_ID, "Sheet1!A2:E19", &valueRange);
  if (success) {
    Serial.println("Data uploaded successfully!");
  } else {
    Serial.println("Failed to upload data.");
    Serial.println(GSheet.errorReason());
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
#endif


// 設定我的結果管理員物件:
pillbox_manager pillbox_mgr;



// ===========================
// Enter your WiFi credentials
// ===========================
// Wi-Fi 熱點設定 (依序設定 SSID 和密碼)
const char* wifiNetworks[][2] = {
  {"mimas-dlink-23C6", "0926889555"},
  {"AI-08-2775", "aiclass1234"},
  {"justin_mimas_p30pro", "0926889555"}
};

const int wifiCount = sizeof(wifiNetworks) / sizeof(wifiNetworks[0]);
int connectToWiFi() {
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
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    GSheet.printf("ESP Google Sheet Client v%s\n\n", ESP_GOOGLE_SHEET_CLIENT_VERSION);

  gsheet_status=connectToWiFi();
  
  // 有連上線在將 GSheet建立
  if(gsheet_status == WL_CONNECTED){
    GSheet.setTokenCallback(tokenStatusCallback);
    GSheet.setPrerefreshSeconds(10 * 60);
    GSheet.begin(CLIENT_EMAIL, PROJECT_ID, PRIVATE_KEY);
  }


#if defined(SUEDOINFERENCE)

#else // 跑推論要用到的初始化設定:
    if (!ei_camera_init()) {
        ei_printf("Failed to initialize Camera!\n");
    } else {
        ei_printf("Camera initialized\n");
    }

    pixels.begin();
    pixels.clear();
    for(int i = 0; i < 16; i++) {
        pixels.setPixelColor(i, pixels.Color(68,96,96));
        pixels.show();
        delay(DELAYVAL);
    }
#endif // SUEDOINFERENCE
    ei_printf("Starting continuous inference in 2 seconds...\n");
    delay(2000);
    

}

void loop()
{
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    // 若到第3回合, 暫停30秒 印出物件 pillbox_mgr紀錄的結，並上傳 Google sheet
    if(rounds>MAX_ROUND){
      pillbox_mgr.print_grid();
      uploadToGoogleSheet(pillbox_mgr);
      ei_sleep(60000);
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

    if (millis() - start_time > (cur_grid == 0? DETECT_TIME_THRESHOLD+16000: DETECT_TIME_THRESHOLD) || detect_count >= MAX_DETECTION_COUNT) {
        // 判斷是否要紀錄該格子的結果
        pillbox_mgr.record(detected_pills, cur_grid);
        cur_grid++;  // 移動到下一個格子
        detect_count = 0; // 重置計數
        return;
    }

#if defined(SUEDOINFERENCE)
  // 不跑推論, 用亂數模擬
  simulated_detection(detected_pills);
  detect_count++;

#else //跑Edge Impulse推論 run_classifier:
    
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
        if (bb.value >= CLASS_CONFIDENCE) {
            detected_pills.push_back(bb);
            // detected_pills[i]=bb;
        }
    }

    detect_count++;  // 增加偵測次數
    free(snapshot_buf);
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

/**
 * @brief 擷取影像並裁剪當前 grid 的 ROI
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);
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
#endif //SUEDOINFERENCE


