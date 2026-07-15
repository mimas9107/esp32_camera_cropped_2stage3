---
name:          README.md
description:   ESP32-CAM 藥丸視覺辨識固件 — Edge Impulse TinyML 推論 + Google Sheets 上傳
created_date:  2026/07/15 14:00:00
modified_date: 2026/07/15 14:00:00
project_version: 1.00.00
document_version: 1.0.0
agent_sign: ['human/mimas', 'opencode/big-pickle']
---

# ESP32-CAM 智慧藥盒固件

ESP32-CAM 上運行的 AI 藥丸辨識固件。以 Edge Impulse FOMO 模型逐格偵測 18 格藥盒中的藥丸，偵測結果上傳至 Google Sheets。

## 功能

- **AI 推論**：Edge Impulse FOMO 模型，96x96 輸入，逐格偵測
- **排程省電**：依照 `detectTimes[]` 定時喚醒，非工作時間 deep sleep
- **雲端上傳**：偵測結果 + 時間戳 + 服藥狀態寫入 Google Sheets
- **模擬模式**：`SUEDOINFERENCE` 開關可用亂數取代真實推論，方便流程驗證

## 硬體需求

- ESP32-CAM 開發板 (OV2640)
- WS2812 燈條 (16 顆, GPIO 12)
- LED x3 (GPIO 2, 13, 14)

## 建置

1. Arduino IDE 2.x + ESP32 Board Package
2. 建立 `credential.h`：

```cpp
const char* wifiNetworks[][2] = {
  {"YourSSID", "YourPassword"},
};
const char* CLIENT_EMAIL = "your-service-account@project.iam.gserviceaccount.com";
const char* PROJECT_ID = "your-project-id";
const char* PRIVATE_KEY = "-----BEGIN PRIVATE KEY-----\n...\n-----END PRIVATE KEY-----";
const char* GOOGLE_SHEET_ID = "your-spreadsheet-id";
```

3. 安裝所需函式庫：`Adafruit NeoPixel`, `ESP Google Sheet Client`
4. 上傳 `esp32_camera_cropped_2stage3.ino`

## 更新紀錄

- **2025/03/17** — 排程判斷、燈號系統、Google Sheet 上傳欄位修正
- **2025/03/10** — Google Sheet 資料上傳空值補齊、重試機制

## 相關

- [根目錄 README](../README.md)
- [技術規格書](../SPEC.md)
