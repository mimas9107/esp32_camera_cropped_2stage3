## 智慧藥盒 esp32 cam AI視覺辨識程式碼

### 2025/03/17 
1. 排程判斷: isDetectTime(), getNextScheduleInterval(), 直接以 ei_sleep實作
2. 燈號判斷: LEDPIN, INITLED, IDLELED
3. 上傳google sheet資料邏輯修正: 解掉 clear清不乾淨的邏輯, 新增 服藥 "Taken", 時間戳記 "Timestamp", 日期 "Date".
4.

### 2025/03/10~3/11
1. 上傳 google sheet資料邏輯修正: 空值也要上傳、上傳失敗的重試