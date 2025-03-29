#include <Wire.h>
#include <TFT_eSPI.h>
#include "EEPROM.h"
#include <SI4735.h>
#include "Rotary.h"
#include "patch_init.h"  // SSB patch for whole SSBRX initialization string
#include "HanYiZhongYuanJian16.h"
#include "orbitronbold16.h"
#include "icon.h"
#include "FREQFONT.h"
#include "NotoMono16.h"
#include "esp_sleep.h"
//#include "driver/adc.h"
#include "Battery.h"
const uint16_t size_content = sizeof ssb_patch_content;  // see patch_init.h

#define FM_BAND_TYPE 0
#define MW_BAND_TYPE 1
#define SW_BAND_TYPE 2
#define LW_BAND_TYPE 3

#define PIN_POWER_ON 15
#define PIN_LCD_BL 38
#define RESET_PIN 16  // GPIO16
#define AUDIO_MUTE 3  // GPIO3
#define AMP_EN 10

// Enconder PINs
#define ENCODER_PIN_A 2  // GPIO02
#define ENCODER_PIN_B 1  // GPIO01

// I2C bus pin on Lilygo T-Display
#define ESP32_I2C_SDA 18  // GPIO18
#define ESP32_I2C_SCL 17  // GPIO17

//#define VBAT_MON 4  // GPIO04
#define MIN_USB_VOLTAGE 4.35
double  CONV_FACTOR=1.71; //开机时加载用, 从flash中重新读取校准值

// Buttons controllers
#define ENCODER_PUSH_BUTTON 21  // GPIO21

#define MIN_ELAPSED_RSSI_TIME 20  //200
#define ELAPSED_COMMAND 5000      // time to turn off the last command controlled by encoder. Time to goes back to the FVO control
#define DEFAULT_VOLUME 35         // change it for your favorite sound volume
#define RDS_CHECK_TIME 90

#define FM 0
#define LSB 1
#define USB 2
#define AM 3
#define LW 4

#define SSB 1

#define VOLUME 0
#define STEP 1
#define MODE 2
#define BFO 3
#define BW 4
#define AGC_ATT 5
#define SOFTMUTE 6
#define BAND 7
#define MUTE 8
#define bright 9
#define about 10
#define save 11
#define selstation 12
#define autolcdoff 13

#define TFT_MENU_BACK TFT_BLACK  // 0x01E9
#define TFT_MENU_HIGHLIGHT_BACK TFT_BLUE

#define EEPROM_SIZE 2048
#define STORE_TIME 10000  // Time of inactivity to make the current receiver status writable (10s / 10000 milliseconds).

unsigned char brightness = 50;  //存储方式数值，128为中间值
// EEPROM - Stroring control variables
const uint8_t app_id = 47;  // Useful to check the EEPROM content before processing useful data
const int eeprom_address = 0;
long storeTime = millis();
int savenum = 1;                                         //存储选择
int selnum = 1;                                          //读取选择
int station[50];                                         //存储电台数组  100显示不下
const int storenumber = (sizeof station / sizeof(int));  //实际存储数51 从1开始存储

int lcdofftime = 5;  //熄屏时间
bool lcdoffed;

unsigned long lastActiveTime = 0;  // 最后活动时间记录
unsigned long lcdondelaytime = 0;
//电池定时获取百分比
unsigned long lastSampleTime = 0;  // 记录上次电量百分比采样时间
int batteryLevel = 0;              // 存储稳定后的电量值

int event = 0;  //按键事件

//////////////???????
// 新增变量用于记录上一次编码器转动时间
unsigned long lastEncoderTime = 0;

//bool seeking=false;

bool itIsTimeToSave = false;
bool bfoOn = false;
bool ssbLoaded = false;
char bfo[18] = "0000";
bool muted = false;
int8_t agcIdx = 1;       //0-打开自动增益  1-衰减0
uint8_t disableAgc = 1;  //0
int8_t agcNdx = 0;
int8_t softMuteMaxAttIdx = 0;

uint8_t seekDirection = 1;

bool cmdBand = false;
bool cmdVolume = false;
bool cmdAgc = false;
bool cmdBandwidth = false;
bool cmdStep = false;
bool cmdMode = false;
bool cmdMenu = false;
bool cmdSoftMuteMaxAtt = false;
bool cmdbright = false;
bool cmdabout = false;
bool fmRDS = false;
bool cmdselstation = false;
bool cmdsave = false;
bool cmdseek = false;
bool cmdautolcdoff = false;

int16_t currentBFO = 0;
long elapsedRSSI = millis();
long lastRDSCheck = millis();
long elapsedCommand = millis();
volatile int encoderCount = 0;
uint16_t currentFrequency;

char sAgc[15];
int8_t menuIdx = VOLUME;
int8_t currentMenuCmd = -1;
const uint8_t currentBFOStep = 10;
const char *menu[] = { "音量", "步进", "模式", "拍频", "带宽", "增益", "降噪", "波段", "静音", "亮度", "关于", "存储电台", "读取电台", "自动熄屏" };  //增加亮度... , 删除"上搜索", "下搜索"
const int lastMenu = (sizeof menu / sizeof(char *)) - 1;

typedef struct
{
  uint8_t idx;       // SI473X device bandwidth index
  const char *desc;  // bandwidth description
} Bandwidth;

int8_t bwIdxSSB = 4;
const int8_t maxSsbBw = 5;
Bandwidth bandwidthSSB[] = {
  { 4, "0.5" },
  { 5, "1.0" },
  { 0, "1.2" },
  { 1, "2.2" },
  { 2, "3.0" },
  { 3, "4.0" }
};
const int lastBandwidthSSB = (sizeof bandwidthSSB / sizeof(Bandwidth)) - 1;

int8_t bwIdxAM = 4;
const int8_t maxAmBw = 6;
Bandwidth bandwidthAM[] = {
  { 4, "1.0" },
  { 5, "1.8" },
  { 3, "2.0" },
  { 6, "2.5" },
  { 2, "3.0" },
  { 1, "4.0" },
  { 0, "6.0" }
};
const int lastBandwidthAM = (sizeof bandwidthAM / sizeof(Bandwidth)) - 1;

int8_t bwIdxFM = 0;
const int8_t maxFmBw = 4;

Bandwidth bandwidthFM[] = {
  { 0, "AUT" },  // Automatic - default
  { 1, "110" },  // Force wide (110 kHz) channel filter.
  { 2, " 84" },
  { 3, " 60" },
  { 4, " 40" }
};
const int lastBandwidthFM = (sizeof bandwidthFM / sizeof(Bandwidth)) - 1;

int tabAmStep[] = { 1, 9, 50, 100 };  //am步进
const int lastAmStep = (sizeof tabAmStep / sizeof(int)) - 1;
int idxAmStep = 3;

int tabFmStep[] = { 10, 20, 100 };  //fm步进
const int lastFmStep = (sizeof tabFmStep / sizeof(int)) - 1;
int idxFmStep = 1;

int tabSwStep[] = { 1, 5, 10, 50, 100 };  //sw步进
const int lastSwStep = (sizeof tabSwStep / sizeof(int)) - 1;
int idxSwStep = 1;


uint16_t currentStepIdx = 1;


const char *bandModeDesc[] = { "FM ", "LSB", "USB", "AM " };
const int lastBandModeDesc = (sizeof bandModeDesc / sizeof(char *)) - 1;
uint8_t currentMode = FM;


/**
 *  Band data structure
 */
typedef struct
{
  const char *bandName;   // Band description
  uint8_t bandType;       // Band type (FM, MW or SW)
  uint16_t minimumFreq;   // Minimum frequency of the band
  uint16_t maximumFreq;   // maximum frequency of the band
  uint16_t currentFreq;   // Default frequency or current frequency
  int8_t currentStepIdx;  // Idex of tabStepAM:  Defeult frequency step (See tabStepAM)
  int8_t bandwidthIdx;    // Index of the table bandwidthFM, bandwidthAM or bandwidthSSB;
} Band;


Band band[] = {
  { "VHF", FM_BAND_TYPE, 6400, 10800, 10390, 0, 0 },
  { "MW1", MW_BAND_TYPE, 153, 1719, 810, 1, 4 },
  { "MW2", MW_BAND_TYPE, 531, 1701, 783, 1, 4 },
  { "MW3", MW_BAND_TYPE, 1701, 3501, 2500, 1, 4 },
  { "80M", SW_BAND_TYPE, 3500, 4000, 3700, 1, 4 },  //原为 MW_BAND_TYPE
  { "SW1", SW_BAND_TYPE, 4000, 5500, 4885, 1, 4 },
  { "SW2", SW_BAND_TYPE, 5500, 6500, 6000, 1, 4 },
  { "40M", SW_BAND_TYPE, 6500, 7300, 7100, 1, 4 },
  { "SW3", SW_BAND_TYPE, 7200, 8000, 7200, 1, 4 },
  { "SW4", SW_BAND_TYPE, 9000, 11000, 9500, 1, 4 },
  { "SW5", SW_BAND_TYPE, 11100, 13000, 11900, 1, 4 },
  { "SW6", SW_BAND_TYPE, 13000, 14000, 13500, 1, 4 },
  { "20M", SW_BAND_TYPE, 14000, 15000, 14200, 1, 4 },
  { "SW7", SW_BAND_TYPE, 15000, 17000, 15300, 1, 4 },
  { "SW8", SW_BAND_TYPE, 17000, 18000, 17500, 1, 4 },
  { "15M", SW_BAND_TYPE, 20000, 21400, 21100, 1, 4 },
  { "SW9", SW_BAND_TYPE, 21400, 22800, 21500, 1, 4 },
  { "CB ", SW_BAND_TYPE, 26000, 28000, 27500, 1, 4 },
  { "10M", SW_BAND_TYPE, 28000, 30000, 28400, 1, 4 },
  { "ALL", SW_BAND_TYPE, 150, 30000, 15000, 1, 4 }  // All band. LW, MW and SW (from 150kHz to 30MHz)
};

const int lastBand = (sizeof band / sizeof(Band)) - 1;
int bandIdx = 0;

char *rdsMsg;
char *stationName;
char *rdsTime;
char bufferStationName[50];
char bufferRdsMsg[100];
char bufferRdsTime[32];

uint8_t rssi = 0;
uint8_t snr = 0;
uint8_t volume = DEFAULT_VOLUME;

int XbatPos = 292;  // Position of battery icon
int YbatPos = 4;
int Xbatsiz = 20;  // size of battery icon
int Ybatsiz = 11;
int previousBatteryLevel = -1;
int currentBatteryLevel = 1;
bool batteryCharging = false;

int leftrect_x=1;
int leftrect_y=20;
int rect_w=76;
int rect_h=22;
int space=25;


// Devices class declarations
Rotary encoder = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);
//Battery18650Stats battery(VBAT_MON, CONV_FACTOR);

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

SI4735 rx;
//BluetoothSerial SerialBT;





void setup() {
  //SerialBT.begin("SI4732-MINI"); // 蓝牙设备名称

  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  Wire.begin(ESP32_I2C_SDA, ESP32_I2C_SCL);

  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, LOW);

  tft.begin();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(3);
  spr.setSwapBytes(true);
  spr.createSprite(320, 170);

  ledcSetup(0, 15000, 8);        //频率提高后背光底噪消失 原频率2000
  ledcAttachPin(PIN_LCD_BL, 0);  //esp 2.x

  //ledcAttach(PIN_LCD_BL, 15000, LEDC_TIMER_8_BIT); //背光初始化

  // Encoder pins
  pinMode(ENCODER_PUSH_BUTTON, INPUT_PULLUP);
  
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), rotaryEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), rotaryEncoder, CHANGE);

  pinMode(AMP_EN, OUTPUT);

  rx.setI2CFastModeCustom(100000);

  int16_t si4735Addr = rx.getDeviceI2CAddress(RESET_PIN);  // Looks for the I2C bus address and set it.  Returns 0 if error

  if (si4735Addr == 0) {
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("Si4735 not detected");
    while (1) ;
  }
  rx.setup(RESET_PIN, FM_BAND_TYPE);
  // Comment the line above and uncomment the three lines below if you are using external ref clock (active crystal or signal generator)
  //rx.setRefClock(32768);
  //rx.setRefClockPrescaler(1);   // will work with 32768
  //rx.setup(RESET_PIN, 0, MW_BAND_TYPE, SI473X_ANALOG_AUDIO, XOSCEN_RCLK);

  cleanBfoRdsInfo();

  delay(100);  //300

  EEPROM.begin(EEPROM_SIZE);
  
  CONV_FACTOR =double( ( EEPROM.read(eeprom_address + 10) << 8 | EEPROM.read(eeprom_address + 11) ) / 1000.0  );    //读取系数高八位

  // Checking the EEPROM content
  if (EEPROM.read(eeprom_address) == app_id) {
    readAllReceiverInformation();
  } else
    rx.setVolume(volume);

  useBand();
  showStatus();
  drawSprite();
  digitalWrite(AMP_EN, HIGH);
  rx.setAudioMuteMcuPin(AUDIO_MUTE);

  if (brightness != 0) {
    ledcWrite(0, brightness);
  } else ledcWrite(0, 50);
  
  if(CONV_FACTOR>10) CONV_FACTOR=1.7;

  batteryLevel = getBatteryChargeLevel(CONV_FACTOR);
}
///////////////////////////////////////////////////////////////以上setup







/**
 * Prints a given content on display 
 */
void print(uint8_t col, uint8_t lin, const GFXfont *font, uint8_t textSize, const char *msg) {
  tft.setCursor(col, lin);
  tft.setTextSize(textSize);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.println(msg);
}

void printParam(const char *msg) {
  tft.fillScreen(TFT_BLACK);
  print(0, 10, NULL, 2, msg);
}

/**
 * 检测按钮事件（单击、双击、三击、长按）
 * @param buttonPin      按钮引脚
 * @param debounceTime   防抖时间（默认50ms）
 * @param longPressTime  长按阈值（默认1000ms）
 * @param multiClickTimeout 多击超时时间（默认300ms）
 * @return 事件编码：
 *   0 = 无事件
 *   1 = 单击
 *   2 = 双击
 *   3 = 三击
 *   4 = 长按
 */
int getButtonEvent(int buttonPin, unsigned long debounceTime = 50, 
                  unsigned long shortPressTime = 200,  
                  unsigned long longPressTime = 800,
                  unsigned long multiClickTimeout = 300) {
  static unsigned long lastPressTime = 0;
  static unsigned long lastReleaseTime = 0;
  static int clickCount = 0;
  static bool buttonActive = false;
  static bool longPressTriggered = false;

  int event = 0;
  unsigned long currentTime = millis();
  bool buttonState = digitalRead(buttonPin);

  // 处理按钮按下
  if (buttonState == LOW) {
    if (!buttonActive && (currentTime - lastReleaseTime > debounceTime)) {
      buttonActive = true;
      lastPressTime = currentTime;
      longPressTriggered = false; // 重置长按标志
    }

    // 持续按压时检测长按
    if (buttonActive && !longPressTriggered) {
      if (currentTime - lastPressTime >= longPressTime) {
        longPressTriggered = true;
        event = 6;               // 长按事件
        clickCount = 0;          // 清除点击计数
      }
    }
  }
  // 处理按钮释放
  else {
    if (buttonActive) {
      buttonActive = false;
      unsigned long pressDuration = currentTime - lastPressTime;
      
      /* 短长按判断（释放时触发）*/
      if (!longPressTriggered && pressDuration >= shortPressTime) {
        event = 5;               // 短长按事件
        clickCount = 0;          // 清除点击计数
      }
      /* 单击计数（未达到短长按时）*/
      else if (!longPressTriggered) {
        clickCount++;
      }
      
      longPressTriggered = false; // 重置长按标志
      lastReleaseTime = currentTime;
    }

    // 处理多击超时
    if (!buttonActive && clickCount > 0 && 
       (currentTime - lastReleaseTime >= multiClickTimeout)) {
      event = (clickCount <= 4) ? clickCount : 4;
      clickCount = 0;
    }
  }

  return event;
}

// bool checkButtonPress(int buttonPin, unsigned long debounceDelay, unsigned long holdDelay, unsigned long ignoreDisconnectTime) {
//   static int buttonState = HIGH;
//   static int lastButtonState = HIGH;
//   static unsigned long lastDebounceTime = 0;
//   static unsigned long lastPressTime = 0;
//   static unsigned long disconnectStartTime = 0;
//   static bool isButtonPressed = false;
//   static bool wasPressed = false;
//
//   int reading = digitalRead(buttonPin);
//
//   // 如果按键状态发生变化，记录当前时间
//   if (reading != lastButtonState) {
//     lastDebounceTime = millis();
//   }
//
//   // 如果经过消抖时间后按键状态仍然稳定
//   if ((millis() - lastDebounceTime) > debounceDelay) {
//     if (reading != buttonState) {
//       buttonState = reading;
//
//       if (buttonState == LOW) {  // 按键按下
//         isButtonPressed = true;
//         lastPressTime = millis();
//         wasPressed = true;
//       } else {  // 按键松开
//         if (wasPressed) {
//           disconnectStartTime = millis();
//         }
//       }
//     }
//   }
//
//   // 处理按键长时间按下时短暂断开的情况
//   if (wasPressed && buttonState == HIGH && (millis() - disconnectStartTime) < ignoreDisconnectTime) {
//     isButtonPressed = true;
//   } else if (buttonState == HIGH) {
//     if ((millis() - lastPressTime) < holdDelay) {
//       isButtonPressed = true;
//     } else {
//       isButtonPressed = false;
//       wasPressed = false;
//     }
//   }
//
//   // 更新上一次的按键状态
//   lastButtonState = reading;
//
//   return isButtonPressed;
// }

void savestation(int n)  //存储电台
{
  int addr_offset;

  EEPROM.begin(EEPROM_SIZE);

  EEPROM.write(eeprom_address, app_id);                                           // stores the app id;
  EEPROM.write(eeprom_address + 100 + n * 15 + 1, bandIdx);                       // Stores the current band
  EEPROM.write(eeprom_address + 100 + n * 15 + 2, currentMode);                   // Stores the current Mode (FM / AM / SSB)
  EEPROM.write(eeprom_address + 100 + n * 15 + 3, (rx.getFrequency() >> 8));      // stores the current Frequency HIGH byte for the band
  EEPROM.write(eeprom_address + 100 + n * 15 + 4, (rx.getFrequency() & 0xFF));    //stores the current Frequency LOW byte for the band
  EEPROM.write(eeprom_address + 100 + n * 15 + 5, band[bandIdx].currentStepIdx);  // Stores current step of the band
  EEPROM.write(eeprom_address + 100 + n * 15 + 6, band[bandIdx].bandwidthIdx);    // table index (direct position) of bandwidth
  EEPROM.write(eeprom_address + 100 + n * 15 + 7, savenum);                       //存储当前电台号码

  EEPROM.commit();
  EEPROM.end();
}

void readstation(int n)  //读取电台
{
  EEPROM.begin(EEPROM_SIZE);

  bandIdx = EEPROM.read(eeprom_address + 100 + n * 15 + 1);
  currentMode = EEPROM.read(eeprom_address + 100 + n * 15 + 2);
  band[bandIdx].currentFreq = EEPROM.read(eeprom_address + 100 + n * 15 + 3) << 8;
  band[bandIdx].currentFreq |= EEPROM.read(eeprom_address + 100 + n * 15 + 4);
  band[bandIdx].currentStepIdx = EEPROM.read(eeprom_address + 100 + n * 15 + 5);
  band[bandIdx].bandwidthIdx = EEPROM.read(eeprom_address + 100 + n * 15 + 6);


  EEPROM.end();

  currentFrequency = band[bandIdx].currentFreq;

  if (band[bandIdx].bandType == FM_BAND_TYPE) {
    currentStepIdx = idxFmStep = band[bandIdx].currentStepIdx;
    rx.setFrequencyStep(tabFmStep[currentStepIdx]);
  } else {
    currentStepIdx = idxAmStep = band[bandIdx].currentStepIdx;
    rx.setFrequencyStep(tabAmStep[currentStepIdx]);
  }

  int bwIdx = band[bandIdx].bandwidthIdx;

  if (currentMode == LSB || currentMode == USB) {
    loadSSB();
    bwIdxSSB = (bwIdx > 5) ? 5 : bwIdx;
    rx.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
    // If audio bandwidth selected is about 2 kHz or below, it is recommended to set Sideband Cutoff Filter to 0.
    if (bandwidthSSB[bwIdxSSB].idx == 0 || bandwidthSSB[bwIdxSSB].idx == 4 || bandwidthSSB[bwIdxSSB].idx == 5)
      rx.setSSBSidebandCutoffFilter(0);
    else
      rx.setSSBSidebandCutoffFilter(1);
    rx.setSSBBfo(currentBFO);
  } else if (currentMode == AM) {
    bwIdxAM = bwIdx;
    rx.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
  } else {
    bwIdxFM = bwIdx;
    rx.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
  }

  if (currentBFO > 0)
    sprintf(bfo, "+%4.4d", currentBFO);
  else
    sprintf(bfo, "%4.4d", currentBFO);

  useBand();

  delay(50);
}

void saveAllReceiverInformation()  //writes the conrrent receiver information into the eeprom.
{
  int addr_offset;

  EEPROM.begin(EEPROM_SIZE);

  EEPROM.write(eeprom_address, app_id);              // stores the app id;
  EEPROM.write(eeprom_address + 1, rx.getVolume());  // stores the current Volume
  EEPROM.write(eeprom_address + 2, bandIdx);         // Stores the current band
  EEPROM.write(eeprom_address + 3, fmRDS);
  EEPROM.write(eeprom_address + 4, currentMode);  // Stores the current Mode (FM / AM / SSB)
  EEPROM.write(eeprom_address + 5, currentBFO >> 8);
  EEPROM.write(eeprom_address + 6, currentBFO & 0XFF);
  EEPROM.write(eeprom_address + 7, brightness);
  EEPROM.write(eeprom_address + 8, lcdofftime);
  EEPROM.write(eeprom_address + 9, cmdseek ? 1 : 0);  //存储 搜索
  //if(!cmdseek) EEPROM.write(eeprom_address + 79,0);
  //电池校准系数 10


  EEPROM.commit();

  addr_offset = 15;
  band[bandIdx].currentFreq = currentFrequency;

  for (int i = 0; i <= lastBand; i++) {
    EEPROM.write(addr_offset++, (band[i].currentFreq >> 8));    // stores the current Frequency HIGH byte for the band
    EEPROM.write(addr_offset++, (band[i].currentFreq & 0xFF));  // stores the current Frequency LOW byte for the band
    EEPROM.write(addr_offset++, band[i].currentStepIdx);        // Stores current step of the band
    EEPROM.write(addr_offset++, band[i].bandwidthIdx);          // table index (direct position) of bandwidth
    EEPROM.commit();
  }

  EEPROM.end();
}

void readAllReceiverInformation()  //reads the last receiver status from eeprom.
{
  uint8_t volume;
  int addr_offset;
  int bwIdx;
  EEPROM.begin(EEPROM_SIZE);

  volume = EEPROM.read(eeprom_address + 1);  // Gets the stored volume;
  bandIdx = EEPROM.read(eeprom_address + 2);
  fmRDS = EEPROM.read(eeprom_address + 3);
  currentMode = EEPROM.read(eeprom_address + 4);
  currentBFO = EEPROM.read(eeprom_address + 5) << 8;
  currentBFO |= EEPROM.read(eeprom_address + 6);
  brightness = EEPROM.read(eeprom_address + 7);
  lcdofftime = EEPROM.read(eeprom_address + 8);
  cmdseek = bool((EEPROM.read(eeprom_address + 9)));
  //电池校准系数 10

  addr_offset = 15;
  for (int i = 0; i <= lastBand; i++) {
    band[i].currentFreq = EEPROM.read(addr_offset++) << 8;
    band[i].currentFreq |= EEPROM.read(addr_offset++);
    band[i].currentStepIdx = EEPROM.read(addr_offset++);
    band[i].bandwidthIdx = EEPROM.read(addr_offset++);
  }

  EEPROM.end();

  currentFrequency = band[bandIdx].currentFreq;

  if (band[bandIdx].bandType == FM_BAND_TYPE) {
    currentStepIdx = idxFmStep = band[bandIdx].currentStepIdx;
    rx.setFrequencyStep(tabFmStep[currentStepIdx]);
  } else if (band[bandIdx].bandType == MW_BAND_TYPE) {
    currentStepIdx = idxAmStep = band[bandIdx].currentStepIdx;
    rx.setFrequencyStep(tabAmStep[currentStepIdx]);
  } else if (band[bandIdx].bandType == SW_BAND_TYPE){
    currentStepIdx = idxSwStep = band[bandIdx].currentStepIdx;
    rx.setFrequencyStep(tabSwStep[currentStepIdx]);
  }
  

  bwIdx = band[bandIdx].bandwidthIdx;

  if (currentMode == LSB || currentMode == USB) {
    loadSSB();
    bwIdxSSB = (bwIdx > 5) ? 5 : bwIdx;
    rx.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
    // If audio bandwidth selected is about 2 kHz or below, it is recommended to set Sideband Cutoff Filter to 0.
    if (bandwidthSSB[bwIdxSSB].idx == 0 || bandwidthSSB[bwIdxSSB].idx == 4 || bandwidthSSB[bwIdxSSB].idx == 5)
      rx.setSSBSidebandCutoffFilter(0);
    else
      rx.setSSBSidebandCutoffFilter(1);
    rx.setSSBBfo(currentBFO);
  } else if (currentMode == AM) {
    bwIdxAM = bwIdx;
    rx.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
  } else {
    bwIdxFM = bwIdx;
    rx.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
  }

  if (currentBFO > 0)
    sprintf(bfo, "+%4.4d", currentBFO);
  else
    sprintf(bfo, "%4.4d", currentBFO);

  delay(50);
  rx.setVolume(volume);
}

void resetEepromDelay()  //To store any change into the EEPROM, it is needed at least STORE_TIME  milliseconds of inactivity.
{
  elapsedCommand = storeTime = millis();
  itIsTimeToSave = true;
}

void disableCommands()  //Set all command flags to false //When all flags are disabled (false), the encoder controls the frequency
{
  cmdBand = false;
  bfoOn = false;
  cmdVolume = false;
  cmdselstation = false;
  cmdsave = false;
  cmdbright = false;
  cmdautolcdoff = false;
  //cmdabout = false; //不创建菜单
  cmdAgc = false;
  cmdBandwidth = false;
  cmdStep = false;
  cmdMode = false;
  cmdMenu = false;
  cmdSoftMuteMaxAtt = false;

  // showCommandStatus((char *) "VFO ");
}

/**
 * Reads encoder via interrupt
 * Use Rotary.h and  Rotary.cpp implementation to process encoder via interrupt
 * if you do not add ICACHE_RAM_ATTR declaration, the system will reboot during attachInterrupt call. 
 * With ICACHE_RAM_ATTR macro you put the function on the RAM.
 */
ICACHE_RAM_ATTR void rotaryEncoder()  //编码器中断 // rotary encoder events
{
  uint8_t encoderStatus = encoder.process();
  if (encoderStatus)
    encoderCount = (encoderStatus == DIR_CW) ? 1 : -1;
}

void showFrequency()  //Shows frequency information on Display
{
  char tmp[15];
  sprintf(tmp, "%5.5u", currentFrequency);
  drawSprite();
}

void showStatus()  //Shows some basic information on display
{
  showFrequency();
  showRSSI();
}

void showRSSI()  //Shows the current RSSI and SNR status
{
  char sMeter[10];
  sprintf(sMeter, "S:%d ", rssi);
  drawSprite();
}

void showAgcAtt()  //Shows the current AGC and Attenuation status
{
  // lcd.clear();
  rx.getAutomaticGainControl();
  if (agcNdx == 0 && agcIdx == 0)
    strcpy(sAgc, "AGC ON");
  else
    sprintf(sAgc, "衰减: %2.2d", agcNdx);

  drawSprite();
}

void showBFO()  //Shows the current BFO value
{

  if (currentBFO > 0)
    sprintf(bfo, "+%4.4d", currentBFO);
  else
    sprintf(bfo, "%4.4d", currentBFO);
  drawSprite();
  elapsedCommand = millis();
}

void setBand(int8_t up_down)  //Sets Band up (1) or down (!1)
{
  band[bandIdx].currentFreq = currentFrequency;
  band[bandIdx].currentStepIdx = currentStepIdx;
  if (up_down == -1)
    bandIdx = (bandIdx < lastBand) ? (bandIdx + 1) : 0;
  else
    bandIdx = (bandIdx > 0) ? (bandIdx - 1) : lastBand;
  useBand();
  elapsedCommand = millis();
}

void useBand()  //Switch the radio to current band
{
  if (band[bandIdx].bandType == FM_BAND_TYPE) {
    currentMode = FM;
    rx.setTuneFrequencyAntennaCapacitor(0);
    rx.setFM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, tabFmStep[band[bandIdx].currentStepIdx]);
    rx.setSeekFmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
    rx.setSeekFmRssiThreshold(6);  //增加fm搜索rssi阈值  15-25
    rx.setSeekFmSNRThreshold(2);   //增加fm搜索snr阈值   3-8
    rx.setSeekFmSpacing(10);
    bfoOn = ssbLoaded = false;
    bwIdxFM = band[bandIdx].bandwidthIdx;
    rx.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
    rx.setFMDeEmphasis(1);
    rx.RdsInit();
    rx.setRdsConfig(1, 2, 2, 2, 2);
  } else if (band[bandIdx].bandType == SW_BAND_TYPE) {
    // set the tuning capacitor for SW or MW/LW 只留下sw
    rx.setTuneFrequencyAntennaCapacitor(1);  //( band[bandIdx].bandType == MW_BAND_TYPE ||band[bandIdx].bandType == LW_BAND_TYPE) ? 0 : 1
    if (ssbLoaded) {
      rx.setSSB(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, tabSwStep[band[bandIdx].currentStepIdx], currentMode);
      rx.setSSBAutomaticVolumeControl(1);
      rx.setSsbSoftMuteMaxAttenuation(softMuteMaxAttIdx);  // Disable Soft Mute for SSB
      bwIdxSSB = band[bandIdx].bandwidthIdx;
      rx.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
    } else {
      currentMode = AM;
      rx.setAM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, tabSwStep[band[bandIdx].currentStepIdx]);
      bfoOn = false;
      bwIdxAM = band[bandIdx].bandwidthIdx;
      rx.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
      rx.setAmSoftMuteMaxAttenuation(softMuteMaxAttIdx);  // Soft Mute for AM or SSB
    }
    rx.setAutomaticGainControl(disableAgc, agcNdx);
    rx.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);  // Consider the range all defined current band
    rx.setSeekAmSpacing(5);                                                    // Max 10kHz for spacing

  } else if (band[bandIdx].bandType == MW_BAND_TYPE) {
    // set the tuning capacitor for SW or MW/LW 只留下sw
    rx.setTuneFrequencyAntennaCapacitor(0);  //( band[bandIdx].bandType == MW_BAND_TYPE ||band[bandIdx].bandType == LW_BAND_TYPE) ? 0 : 1

    currentMode = AM;
    rx.setAM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, tabAmStep[band[bandIdx].currentStepIdx]);
    bfoOn = false;
    bwIdxAM = band[bandIdx].bandwidthIdx;
    rx.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
    rx.setAmSoftMuteMaxAttenuation(softMuteMaxAttIdx);  // Soft Mute for AM or SSB
    ///////////////////////////////
    rx.setAutomaticGainControl(disableAgc, agcNdx);
    rx.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);  // Consider the range all defined current band
    rx.setSeekAmSpacing(9);                                                    // Max 10kHz for spacing
  }

  //delay(100);
  currentFrequency = band[bandIdx].currentFreq;
  currentStepIdx = band[bandIdx].currentStepIdx;

  rssi = 0;
  snr = 0;
  cleanBfoRdsInfo();
  showStatus();
}

void loadSSB()  //加载单边带文件
{
  rx.setI2CFastModeCustom(400000);  // You can try rx.setI2CFastModeCustom(700000); or greater value
  rx.loadPatch(ssb_patch_content, size_content, bandwidthSSB[bwIdxSSB].idx);
  rx.setI2CFastModeCustom(100000);
  ssbLoaded = true;
}

void doBandwidth(int8_t v)  //Switches the Bandwidth
{
  if (currentMode == LSB || currentMode == USB) {
    bwIdxSSB = (v == 1) ? bwIdxSSB - 1 : bwIdxSSB + 1;

    if (bwIdxSSB > maxSsbBw)
      bwIdxSSB = 0;
    else if (bwIdxSSB < 0)
      bwIdxSSB = maxSsbBw;

    rx.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
    // If audio bandwidth selected is about 2 kHz or below, it is recommended to set Sideband Cutoff Filter to 0.
    if (bandwidthSSB[bwIdxSSB].idx == 0 || bandwidthSSB[bwIdxSSB].idx == 4 || bandwidthSSB[bwIdxSSB].idx == 5)
      rx.setSSBSidebandCutoffFilter(0);
    else
      rx.setSSBSidebandCutoffFilter(1);

    band[bandIdx].bandwidthIdx = bwIdxSSB;
  } else if (currentMode == AM) {
    bwIdxAM = (v == 1) ? bwIdxAM - 1 : bwIdxAM + 1;

    if (bwIdxAM > maxAmBw)
      bwIdxAM = 0;
    else if (bwIdxAM < 0)
      bwIdxAM = maxAmBw;

    rx.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
    band[bandIdx].bandwidthIdx = bwIdxAM;

  } else {
    bwIdxFM = (v == 1) ? bwIdxFM - 1 : bwIdxFM + 1;
    if (bwIdxFM > maxFmBw)
      bwIdxFM = 0;
    else if (bwIdxFM < 0)
      bwIdxFM = maxFmBw;

    rx.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
    band[bandIdx].bandwidthIdx = bwIdxFM;
  }
  drawSprite();
}

void showCommandStatus(char *currentCmd)  //Show cmd on display. It means you are setting up something.
{
  spr.drawString(currentCmd, 38, 14, 2);
  drawSprite();
}

void doAgc(int8_t v)  //AGC and attenuattion setup
{
  agcIdx = (v == 1) ? agcIdx - 1 : agcIdx + 1;
  if (agcIdx < 0)
    agcIdx = 35;
  else if (agcIdx > 35)
    agcIdx = 0;
  disableAgc = (agcIdx > 0);  // if true, disable AGC; esle, AGC is enable
  if (agcIdx > 1)
    agcNdx = agcIdx - 1;
  else
    agcNdx = 0;
  rx.setAutomaticGainControl(disableAgc, agcNdx);  // if agcNdx = 0, no attenuation
  showAgcAtt();
  elapsedCommand = millis();
}

void autostep()  //自动步进
{
  unsigned long currentTime = millis();
  unsigned long timeInterval = currentTime - lastEncoderTime;
  lastEncoderTime = currentTime;

  int8_t stepidx = 0;

  //if      (timeInterval < 110)      stepidx=4;  // 快速转动，步进第三档
  if (timeInterval < 130) stepidx = 3;  // 中速转动，步进第二档
  else stepidx = 0;                     // 恢复最低步进 第一档

  if (currentMode == FM) {
    //currentStepIdx=stepidx
    rx.setFrequencyStep(tabFmStep[currentStepIdx]);  //删除了5k步进
  } else {
    currentStepIdx = stepidx;
    rx.setFrequencyStep(tabAmStep[currentStepIdx]);
  }
  band[bandIdx].currentStepIdx = currentStepIdx;
}

void doStep(int8_t v)  //Switches the current step
{
  if (band[bandIdx].bandType == FM_BAND_TYPE) {
    idxFmStep = (v == 1) ? idxFmStep - 1 : idxFmStep + 1;
    if (idxFmStep > lastFmStep)
      idxFmStep = 0;
    else if (idxFmStep < 0)
      idxFmStep = lastFmStep;

    currentStepIdx = idxFmStep;
    rx.setFrequencyStep(tabFmStep[currentStepIdx]);

  } else if (band[bandIdx].bandType == MW_BAND_TYPE) {
    idxAmStep = (v == 1) ? idxAmStep - 1 : idxAmStep + 1;
    if (idxAmStep > lastAmStep)
      idxAmStep = 0;
    else if (idxAmStep < 0)
      idxAmStep = lastAmStep;

    currentStepIdx = idxAmStep;
    rx.setFrequencyStep(tabAmStep[currentStepIdx]);
  } else if (band[bandIdx].bandType == SW_BAND_TYPE) {
    idxSwStep = (v == 1) ? idxSwStep - 1 : idxSwStep + 1;
    if (idxSwStep > lastSwStep)
      idxSwStep = 0;
    else if (idxSwStep < 0)
      idxSwStep = lastSwStep;

    currentStepIdx = idxSwStep;
    rx.setFrequencyStep(tabSwStep[currentStepIdx]);
  }
  band[bandIdx].currentStepIdx = currentStepIdx;
  drawSprite();  //showStep();
  elapsedCommand = millis();
}

void doMode(int8_t v)  //Switches to the AM, LSB or USB modes
{
  if (currentMode != FM) {
    if (v == 1) {  // clockwise
      if (currentMode == AM) {
        // If you were in AM mode, it is necessary to load SSB patch (avery time)

        spr.fillSmoothRoundRect(80, 40, 160, 40, 4, TFT_WHITE);
        spr.fillSmoothRoundRect(81, 41, 158, 38, 4, TFT_MENU_BACK);
        spr.drawString("Loading SSB", 160, 62, 4);
        spr.pushSprite(0, 0);

        loadSSB();
        ssbLoaded = true;
        currentMode = USB;
      } else if (currentMode == USB)
        currentMode = LSB;
      else if (currentMode == LSB) {
        currentMode = AM;
        bfoOn = ssbLoaded = false;
      }
    } else {  // and counterclockwise
      if (currentMode == AM) {
        // If you were in AM mode, it is necessary to load SSB patch (avery time)

        spr.fillSmoothRoundRect(80, 40, 160, 40, 4, TFT_WHITE);
        spr.fillSmoothRoundRect(81, 41, 158, 38, 4, TFT_MENU_BACK);
        spr.drawString("Loading SSB", 160, 62, 4);
        spr.pushSprite(0, 0);

        loadSSB();
        ssbLoaded = true;
        currentMode = LSB;
      } else if (currentMode == LSB)
        currentMode = USB;
      else if (currentMode == USB) {
        currentMode = AM;
        bfoOn = ssbLoaded = false;
      }
    }
    // Nothing to do if you are in FM mode
    band[bandIdx].currentFreq = currentFrequency;
    band[bandIdx].currentStepIdx = currentStepIdx;
    useBand();
  }
  elapsedCommand = millis();
}

void doVolume(int8_t v)  //调节音量
{
  if (v == 1)
    rx.volumeDown();
  if (v ==-1)
    rx.volumeUp();

  drawSprite();             //showVolume();
}

int readsavednumcount(void)  //将存储的电台号码读取进 station[]数组 返回存储个数count_station
{
  int count_station = 0;
  int value;
  EEPROM.begin(EEPROM_SIZE);
  for (int i = 1; i <= storenumber; i++) {
    value = EEPROM.read(eeprom_address + 100 + i * 15 + 7);  //将存储的savenum 读取出

    if (value != 255)  //初始化默认255
    {
      count_station++;
      station[count_station] = value;
    }
  }
  EEPROM.end();
  return count_station;
}

void doselstation(int8_t v)  //读取电台设置 selnum
{
  if (v == -1) selnum++;
  if (selnum > readsavednumcount()) selnum = 1;
  else if (v == 1) selnum--;
  if (selnum < 1) selnum = readsavednumcount();

  //  tft.drawString(String(readsavednumcount()),160,102,4);  //在rds位置显示调试数据
  //  tft.drawString(String(station[selnum]),190,102,4);  //在rds位置显示调试数据

  readstation(station[selnum]);

  drawSprite();

}

void dosavestation(int8_t v)  //保存电台设置 savenum
{
  if (v == -1) savenum++;
  if (savenum > storenumber) savenum = 1;
  else if (v == 1) savenum--;
  if (savenum < 1) savenum = storenumber;


  drawSprite();
}

void doabout(int8_t v)  //关于 屏幕刷新会刷新掉 暂时添加延时
{
  if (v == 1) {
  } else {
  }
  //spr.createSprite(320, 170);
  spr.fillSprite(TFT_BLACK);
  //spr.setTextColor(TFT_WHITE, TFT_BLACK);
  spr.setTextDatum(ML_DATUM);
  spr.loadFont(HanYiZhongYuanJian16);

  spr.drawString("嘉立创 你别失望 开源 ", 1, 10);
  spr.drawString("oshwhub.com/sunnygold", 1, 30);
  spr.drawString("软件 汉化 改进 :zooc", 1, 50);
  spr.drawString(" ", 1, 70);
  spr.drawString("QQ群:854656412", 1, 90);
  spr.drawFloat(CONV_FACTOR,10, 100,130);

  spr.unloadFont();

  spr.pushSprite(0, 0); 
  while (digitalRead(21));
}

void dobright(int8_t v)  //调节亮度
{

  if (brightness < 1) { brightness = 1; }
  if (brightness > 255) { brightness = 255; }

  if (v == 1) {
    brightness = brightness - 5;
    ledcWrite(0, brightness);
    drawSprite();

  } else {
    brightness = brightness + 5;
    ledcWrite(0, brightness);
    drawSprite();
  }
}

void sellcdofftime(int8_t v)  //自动熄屏时间调节 0-99
{
  if (v == -1) lcdofftime++;
  if (lcdofftime > 99) lcdofftime = 1;
  else if (v == 1) lcdofftime--;
  if (lcdofftime < 0) lcdofftime = 99;

  drawSprite();
}

void checkScreenTimeout()  //检查屏幕超时 如果超过自动熄屏时间 熄屏 在按键编码器事件中通过 lcdoff(false)复位
{
  unsigned long currentTime = millis();
  if (lcdofftime != 0 && (currentTime - lastActiveTime > lcdofftime * 1000)) {  //lcdoff(false) &&
    lcdoff(true);
  }
}

void lcdoff(bool off)  //控制屏幕亮灭 控制lcdoffed变量
{
  if (off) {
    ledcWrite(0, 0); //背光关闭
    //digitalWrite(TFT_PWR, LOW);
    //adc_digi_stop();
    lcdoffed = true;
  }  //关闭背光
  else {
    ledcWrite(0, brightness);
    //adc_digi_start();
    //pinMode(21, INPUT_PULLUP);
    delay(200);        //if (currentTime  - lcdondelaytime >1000 )
    lcdoffed = false;  //亮屏后200ms 执行lcdoffed=false
  }
}

void showFrequencySeek(uint16_t freq)  //显示频率 This function is called by the seek function process
{
  currentFrequency = freq;
  showFrequency();
}

bool stopseek()  //seekStationProgress 回调函数 用于停止搜索
{
  if ((encoderCount == 1 & seekDirection == 1) | (encoderCount == -1 & seekDirection == 0))  //如果编码器方向和搜索方向不一致
  {
    return true;  //返回真 停止搜索
  }
  return false;
}

void doSeek(int8_t v)  //搜索电台
{
  delay(200);  //增加延时 防止反向拧多了没有停止反向搜索

  if (v == 1) seekDirection = 0;
  if (v == -1) seekDirection = 1;

  rx.seekStationProgress(showFrequencySeek, stopseek, seekDirection);  //seekup 1  seek down 0
  currentFrequency = rx.getFrequency();
}

void doSoftMute(int8_t v)  //Sets the Soft Mute Parameter
{
  softMuteMaxAttIdx = (v == 1) ? softMuteMaxAttIdx - 1 : softMuteMaxAttIdx + 1;
  if (softMuteMaxAttIdx > 32)
    softMuteMaxAttIdx = 0;
  else if (softMuteMaxAttIdx < 0)
    softMuteMaxAttIdx = 32;

  rx.setAmSoftMuteMaxAttenuation(softMuteMaxAttIdx);
  drawSprite();  //showSoftMute();
  elapsedCommand = millis();
}

void doMenu(int8_t v)  //Menu options selection
{
  menuIdx = (v == 1) ? menuIdx - 1 : menuIdx + 1;

  if (menuIdx > lastMenu)
    menuIdx = 0;
  else if (menuIdx < 0)
    menuIdx = lastMenu;

  drawSprite();
  elapsedCommand = millis();
}

void doCurrentMenuCmd()  //Starts the MENU action process
{
  disableCommands();
  switch (currentMenuCmd) {
    case VOLUME:  // VOLUME
      cmdVolume = true;
      drawSprite();
      break;
    case save:
      cmdsave = true;
      drawSprite();
      break;
    case selstation:                 //zooc
      if (readsavednumcount() == 0)  //如果未存储电台 显示提示!
      {
        tft.drawString("no station saved!", 110, 90, 2);  //在rds位置显示调试数据
        delay(1000);
      } else {
        cmdselstation = true;
        drawSprite();
      }

      break;
    case about:
      cmdabout = true;
      //关于执行部分
      doabout(encoderCount);
      //
      break;
    case autolcdoff:  //自动熄屏
      cmdautolcdoff = true;
      drawSprite();
      break;
    case bright:  // VOLUME
      cmdbright = true;
      drawSprite();
      break;
    case STEP:  // STEP
      cmdStep = true;
      drawSprite();
      break;
    case MODE:  // MODE
      cmdMode = true;
      drawSprite();
      break;
    case BFO:
      if ((currentMode == LSB || currentMode == USB)) {
        bfoOn = true;
        showBFO();
      }
      showFrequency();
      break;
    case BW:  // BW
      cmdBandwidth = true;
      drawSprite();
      break;
    case AGC_ATT:  // AGC/ATT
      cmdAgc = true;
      showAgcAtt();
      break;
    case SOFTMUTE:
      cmdSoftMuteMaxAtt = true;
      drawSprite();  //showSoftMute();
      break;
    // case SEEKUP:   //删除原搜索
    //   seekDirection = 1;
    //   doSeek(0);
    //   break;
    // case SEEKDOWN:
    //   seekDirection = 0;
    //   doSeek(1);
    //   break;
    case BAND:
      cmdBand = true;
      drawSprite();
      break;
    case MUTE:
      muted = !muted;
      if (muted) {
        rx.setAudioMute(muted);
        digitalWrite(AMP_EN, LOW);
        drawSprite();
      } else {
        rx.setAudioMute(muted);
        digitalWrite(AMP_EN, HIGH);
        drawSprite();
      }
      break;
    default:
      showStatus();
      break;
  }
  currentMenuCmd = -1;
  elapsedCommand = millis();
}

bool isMenuMode()  //Return true if the current status is Menu command
{
  return (cmdMenu | cmdStep | cmdBandwidth | cmdAgc | cmdVolume | cmdSoftMuteMaxAtt | cmdMode | cmdbright | cmdselstation | cmdsave | cmdautolcdoff);
}

uint8_t getStrength() {
  if (currentMode != FM) {
    //dBuV to S point conversion HF
    if ((rssi >= 0) and (rssi <= 1)) return 1;    // S0
    if ((rssi > 1) and (rssi <= 1)) return 2;     // S1
    if ((rssi > 2) and (rssi <= 3)) return 3;     // S2
    if ((rssi > 3) and (rssi <= 4)) return 4;     // S3
    if ((rssi > 4) and (rssi <= 10)) return 5;    // S4
    if ((rssi > 10) and (rssi <= 16)) return 6;   // S5
    if ((rssi > 16) and (rssi <= 22)) return 7;   // S6
    if ((rssi > 22) and (rssi <= 28)) return 8;   // S7
    if ((rssi > 28) and (rssi <= 34)) return 9;   // S8
    if ((rssi > 34) and (rssi <= 44)) return 10;  // S9
    if ((rssi > 44) and (rssi <= 54)) return 11;  // S9 +10
    if ((rssi > 54) and (rssi <= 64)) return 12;  // S9 +20
    if ((rssi > 64) and (rssi <= 74)) return 13;  // S9 +30
    if ((rssi > 74) and (rssi <= 84)) return 14;  // S9 +40
    if ((rssi > 84) and (rssi <= 94)) return 15;  // S9 +50
    if (rssi > 94) return 16;                     // S9 +60
    if (rssi > 95) return 17;                     //>S9 +60
  } else {
    //dBuV to S point conversion FM
    if (rssi < 1) return 1;
    if ((rssi > 1) and (rssi <= 2)) return 7;     // S6
    if ((rssi > 2) and (rssi <= 8)) return 8;     // S7
    if ((rssi > 8) and (rssi <= 14)) return 9;    // S8
    if ((rssi > 14) and (rssi <= 24)) return 10;  // S9
    if ((rssi > 24) and (rssi <= 34)) return 11;  // S9 +10
    if ((rssi > 34) and (rssi <= 44)) return 12;  // S9 +20
    if ((rssi > 44) and (rssi <= 54)) return 13;  // S9 +30
    if ((rssi > 54) and (rssi <= 64)) return 14;  // S9 +40
    if ((rssi > 64) and (rssi <= 74)) return 15;  // S9 +50
    if (rssi > 74) return 16;                     // S9 +60
    if (rssi > 76) return 17;                     //>S9 +60
    // newStereoPilot=si4735.getCurrentPilot();
  }
}

void drawMenu() {
  if (cmdMenu) {
    spr.fillSmoothRoundRect(1, 1, 76, 110, 4, TFT_RED);
    spr.fillSmoothRoundRect(2, 2, 74, 108, 4, TFT_MENU_BACK);
    spr.setTextColor(TFT_WHITE, TFT_MENU_BACK);
    //菜单自定义字体
    spr.loadFont(HanYiZhongYuanJian16);
    spr.drawString("菜单", 38, 14);  //2
    spr.unloadFont();
    spr.setTextFont(0);
    spr.setTextColor(0xBEDF, TFT_MENU_BACK);
    spr.fillRoundRect(6, 24 + (2 * 16), 66, 16, 2, 0x105B);
    for (int i = -2; i < 3; i++) {
      if (i == 0) spr.setTextColor(0xBEDF, 0x105B);
      else spr.setTextColor(0xBEDF, TFT_MENU_BACK);
      //自定义
      spr.loadFont(HanYiZhongYuanJian16);
      spr.drawString(menu[abs((menuIdx + lastMenu + 1 + i) % (lastMenu + 1))], 38, 64 + (i * 16));
      spr.unloadFont();
    }
  } else {
    spr.setTextColor(TFT_WHITE, TFT_MENU_BACK);
    spr.fillSmoothRoundRect(1, 1, 76, 110, 4, TFT_RED);
    spr.fillSmoothRoundRect(2, 2, 74, 108, 4, TFT_MENU_BACK);
    //
    spr.loadFont(HanYiZhongYuanJian16);
    spr.drawString(menu[menuIdx], 38, 14);
    spr.unloadFont();
    //
    spr.setTextFont(0);
    spr.setTextColor(0xBEDF, TFT_MENU_BACK);
    // spr.fillRect(6,24+(2*16),67,16,0xBEDF);
    spr.fillRoundRect(6, 24 + (2 * 16), 66, 16, 2, 0x105B);
    for (int i = -2; i < 3; i++) {
      if (i == 0) spr.setTextColor(0xBEDF, 0x105B);
      else spr.setTextColor(0xBEDF, TFT_MENU_BACK);
      if (cmdMode)
        if (band[bandIdx].bandType == FM_BAND_TYPE) {
          if (i == 0) spr.drawString(bandModeDesc[abs((currentMode + lastBandModeDesc + 1 + i) % (lastBandModeDesc + 1))], 38, 64 + (i * 16), 2);
        } else spr.drawString(bandModeDesc[abs((currentMode + lastBandModeDesc + 1 + i) % (lastBandModeDesc + 1))], 38, 64 + (i * 16), 2);



      if (cmdStep) {
        if (band[bandIdx].bandType == FM_BAND_TYPE) spr.drawNumber(tabFmStep[abs((currentStepIdx + lastFmStep + 1 + i) % (lastFmStep + 1))], 38, 64 + (i * 16), 2);
        else if (band[bandIdx].bandType == SW_BAND_TYPE) spr.drawNumber(tabSwStep[abs((currentStepIdx + lastSwStep + 1 + i) % (lastSwStep + 1))], 38, 64 + (i * 16), 2);
        else if (band[bandIdx].bandType == MW_BAND_TYPE) spr.drawNumber(tabAmStep[abs((currentStepIdx + lastAmStep + 1 + i) % (lastAmStep + 1))], 38, 64 + (i * 16), 2);
      }
      
      spr.loadFont(NotoMono16);
      if (cmdBand) spr.drawString(band[abs((bandIdx + lastBand + 1 + i) % (lastBand + 1))].bandName, 38, 64 + (i * 16), 2);
      spr.unloadFont();
      
      if (cmdBandwidth) {
        if (currentMode == LSB || currentMode == USB) {
          spr.drawString(bandwidthSSB[abs((bwIdxSSB + lastBandwidthSSB + 1 + i) % (lastBandwidthSSB + 1))].desc, 38, 64 + (i * 16), 2);
          // bw = (char *)bandwidthSSB[bwIdxSSB].desc;
          // showBFO();
        } 
        if (band[bandIdx].bandType == MW_BAND_TYPE || SW_BAND_TYPE) {
          spr.drawString(bandwidthAM[abs((bwIdxAM + lastBandwidthAM + 1 + i) % (lastBandwidthAM + 1))].desc, 38, 64 + (i * 16), 2);
        } 
         if (band[bandIdx].bandType == FM_BAND_TYPE){
          spr.drawString(bandwidthFM[abs((bwIdxFM + lastBandwidthFM + 1 + i) % (lastBandwidthFM + 1))].desc, 38, 64 + (i * 16), 2);
        }
      }
    }
    if (cmdVolume) {
      spr.setTextColor(0xBEDF, TFT_MENU_BACK);
      spr.fillRoundRect(6, 24 + (2 * 16), 66, 16, 2, TFT_MENU_BACK);
      spr.drawNumber(rx.getVolume(), 38, 60, 7);
    }
    //增加亮度调节显示
    if (cmdbright) {
      spr.setTextColor(0xBEDF, TFT_MENU_BACK);
      spr.fillRoundRect(6, 24 + (2 * 16), 66, 16, 2, TFT_MENU_BACK);
      spr.drawNumber(brightness / 2.56, 38, 60, 7);  //1-255  /2.56 百分比算
    }
    if (cmdautolcdoff) {
      spr.setTextColor(0xBEDF, TFT_MENU_BACK);
      spr.fillRoundRect(6, 24 + (2 * 16), 66, 16, 2, TFT_MENU_BACK);
      spr.drawNumber(lcdofftime, 38, 60, 7);  //1-255  /2.56 百分比算
    }
    if (cmdsave) {
      //增加已存储判断
      EEPROM.begin(EEPROM_SIZE);
      if (EEPROM.read(eeprom_address + 100 + savenum * 15 + 7) != 255) {
        spr.setTextColor(TFT_RED, TFT_MENU_BACK);
        spr.fillRoundRect(6, 24 + (2 * 16), 66, 16, 2, TFT_MENU_BACK);
        spr.drawNumber(savenum, 38, 60, 7);  //存储电台 号码显示
      } else {
        spr.setTextColor(0xBEDF, TFT_MENU_BACK);
        spr.fillRoundRect(6, 24 + (2 * 16), 66, 16, 2, TFT_MENU_BACK);
        spr.drawNumber(savenum, 38, 60, 7);  //存储电台 号码显示
      }
      EEPROM.end();
    }
    if (cmdselstation) {
      spr.setTextColor(0xBEDF, TFT_MENU_BACK);
      spr.fillRoundRect(6, 24 + (2 * 16), 66, 16, 2, TFT_MENU_BACK);
      spr.drawNumber(station[selnum], 38, 60, 7);  //读取电台 号码显示
    }
    if (cmdAgc) {
      spr.setTextColor(0xBEDF, TFT_MENU_BACK);
      spr.fillRoundRect(6, 24 + (2 * 16), 66, 16, 2, TFT_MENU_BACK);
      rx.getAutomaticGainControl();
      if (agcNdx == 0 && agcIdx == 0) {
        spr.setFreeFont(&Orbitron_Light_24);
        spr.drawString("AGC", 38, 48);
        spr.drawString("On", 38, 72);
        spr.setTextFont(1);
      } else {
        sprintf(sAgc, "%2.2d", agcNdx);
        spr.drawString(sAgc, 38, 60, 7);
      }
    }
    if (cmdSoftMuteMaxAtt) {
      spr.setTextColor(0xBEDF, TFT_MENU_BACK);
      spr.fillRoundRect(6, 24 + (2 * 16), 66, 16, 2, TFT_MENU_BACK);
      spr.drawNumber(softMuteMaxAttIdx, 38, 60, 7);
    }
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
  }
}

void drawSprite() {
  spr.fillSprite(TFT_BLACK);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  
  if (currentMode == FM)  //主界面频率大字 fm
  {
    spr.loadFont(FREQFONT1);
    spr.drawFloat(currentFrequency / 100.00, 1, 158, 60, 7);
    spr.unloadFont();
  } else  //主界面频率大字 am
  {
    spr.loadFont(FREQFONT1);
    spr.drawNumber(currentFrequency, 158, 60, 7);
    spr.unloadFont();
  }

  spr.setFreeFont(&Orbitron_Light_24);
  spr.drawString(band[bandIdx].bandName, 160, 12);  //中上方 波段大字

  if (isMenuMode() or cmdBand) drawMenu();

  else {
    spr.setTextDatum(MC_DATUM);  //水平居中 垂直左对齐

    spr.fillSmoothRoundRect(leftrect_x,   leftrect_y,          rect_w,   rect_h, 4, TFT_WHITE);  //步进边框  x,y,w,h
    spr.fillSmoothRoundRect(leftrect_x+1, leftrect_y+1,        rect_w-2, rect_h-2, 4, TFT_MENU_BACK);
    spr.fillSmoothRoundRect(leftrect_x,   leftrect_y+space*1,  rect_w,   rect_h, 4, TFT_WHITE);  //带宽边框
    spr.fillSmoothRoundRect(leftrect_x+1, leftrect_y+space*1+1,rect_w-2, rect_h-2, 4, TFT_MENU_BACK);
    spr.fillSmoothRoundRect(leftrect_x,   leftrect_y+space*2,  rect_w,   rect_h, 4, TFT_WHITE);  //增益边框
    spr.fillSmoothRoundRect(leftrect_x+1, leftrect_y+space*2+1,rect_w-2, rect_h-2, 4, TFT_MENU_BACK);
    spr.fillSmoothRoundRect(leftrect_x,   leftrect_y+space*3,  rect_w,   rect_h, 4, TFT_WHITE);  //预留 边框
    spr.fillSmoothRoundRect(leftrect_x+1, leftrect_y+space*3+1,rect_w-2, rect_h-2, 4, TFT_MENU_BACK);

    //zooc 自定义字体 通过用到的汉字取模后保存h文件，存档在ino同目录，在ino文件中引用 通过loadfont 和 unloadfont 使用字体
    spr.loadFont(HanYiZhongYuanJian16);
    //spr.setTextColor(spr.color565(200, 200, 200), TFT_MENU_BACK);  //主界面汉字标题部分字体颜色
    //spr.drawString("波段:",5,65+(-3*16));
    //spr.drawString("模式:",5,65+(-2*16));
    spr.drawString("步进", leftrect_x+20, leftrect_y+12);
    spr.drawString("带宽", leftrect_x+20, leftrect_y+12+space);

    if (agcNdx == 0 && agcIdx == 0) {
      spr.loadFont(HanYiZhongYuanJian16);  spr.drawString("增益", leftrect_x+20, leftrect_y+12+space*2);  spr.unloadFont();
      //spr.setTextColor(TFT_WHITE, TFT_BLACK);
      spr.loadFont(NotoMono16);  spr.drawString("ON", leftrect_x+57, leftrect_y+13+space*2, 2);  spr.unloadFont();
    } else {
      sprintf(sAgc, "%2.2d", agcNdx);
      spr.loadFont(HanYiZhongYuanJian16); spr.drawString("衰减", leftrect_x+20, leftrect_y+12+space*2);   spr.unloadFont();
      //spr.setTextColor(TFT_WHITE, TFT_BLACK);
      spr.loadFont(NotoMono16);  spr.drawString(sAgc, leftrect_x+57, leftrect_y+13+space*2, 2);  spr.unloadFont();
    }
    spr.unloadFont();

    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    //步进
    spr.loadFont(NotoMono16);
    if (band[bandIdx].bandType == FM_BAND_TYPE) spr.drawNumber(tabFmStep[currentStepIdx], leftrect_x+57, leftrect_y+13, 2);  //调频 步进
    else if (band[bandIdx].bandType == MW_BAND_TYPE) spr.drawNumber(tabAmStep[currentStepIdx], leftrect_x+57, leftrect_y+13, 2);
    else if (band[bandIdx].bandType == SW_BAND_TYPE) spr.drawNumber(tabSwStep[currentStepIdx], leftrect_x+57, leftrect_y+13, 2);  //区分 sw mw
    //带宽
    if (currentMode == LSB || currentMode == USB) spr.drawString(bandwidthSSB[bwIdxSSB].desc, leftrect_x+57, leftrect_y+13+space, 2);
    else if (currentMode == AM) spr.drawString(bandwidthAM[bwIdxAM].desc, leftrect_x+57, leftrect_y+13+space, 2);  //带宽
    else spr.drawString(bandwidthFM[bwIdxFM].desc, leftrect_x+57, leftrect_y+13+space, 2);                         //带宽
    spr.unloadFont();
    
    spr.setTextDatum(ML_DATUM);  //水平居中 垂直居中
    //拍频
    if (currentMode == LSB || currentMode == USB) {
      spr.loadFont(orbitronbold16);
      spr.drawString("SSB", 1, 11, 4);
      spr.unloadFont();

      spr.setTextDatum(MC_DATUM);
      spr.setTextColor(spr.color565(100, 100, 100), TFT_MENU_BACK);  //!BFO调节 灰色
      spr.drawString("BFO:", 123, 102, 4);                           //BFO 文字下调2
      spr.setTextDatum(MR_DATUM);
      spr.drawString(bfo, 225, 102, 4);
      spr.setTextDatum(MC_DATUM);
    } else  //主界面菜单不显示 只在单边带显示
    {
      spr.setTextColor(spr.color565(100, 100, 100), TFT_MENU_BACK);  //!BFO调节 灰色
      spr.loadFont(orbitronbold16);
      spr.drawString("SSB", 1, 11, 4);
      spr.unloadFont();
    }
    spr.setTextDatum(MC_DATUM);
  }  //以上首页显示 设置和波段界面不显示
  


  spr.fillSmoothRoundRect(leftrect_x+240, leftrect_y,          rect_w,   rect_h,   4, TFT_WHITE);  //模式边框
  spr.fillSmoothRoundRect(leftrect_x+241, leftrect_y+1,        rect_w-2, rect_h-2, 4, TFT_BLACK);
  spr.fillSmoothRoundRect(leftrect_x+240, leftrect_y+space*1,  rect_w,   rect_h,   4, TFT_WHITE);  //信号强度 边框
  spr.fillSmoothRoundRect(leftrect_x+241, leftrect_y+space*1+1,rect_w-2, rect_h-2, 4, TFT_BLACK);
  spr.fillSmoothRoundRect(leftrect_x+240, leftrect_y+space*2,  rect_w,   rect_h,   4, TFT_WHITE);  //音量 边框
  spr.fillSmoothRoundRect(leftrect_x+241, leftrect_y+space*2+1,rect_w-2, rect_h-2, 4, TFT_BLACK);
  spr.fillSmoothRoundRect(leftrect_x+240, leftrect_y+space*3,  rect_w,   rect_h,   4, TFT_WHITE);  //预留 边框
  spr.fillSmoothRoundRect(leftrect_x+241, leftrect_y+space*3+1,rect_w-2, rect_h-2, 4, TFT_BLACK);
  
  spr.drawLine(leftrect_x+243, leftrect_y+3+space, leftrect_x+251, leftrect_y+3+space , TFT_WHITE);  //天线杆
  spr.drawLine(leftrect_x+243, leftrect_y+3+space, leftrect_x+247, leftrect_y+8+space, TFT_WHITE);
  spr.drawLine(leftrect_x+251, leftrect_y+3+space, leftrect_x+247, leftrect_y+8+space, TFT_WHITE);
  spr.drawLine(leftrect_x+247, leftrect_y+3+space, leftrect_x+247, leftrect_y+19+space , TFT_WHITE);

  if (bfoOn) {  //设置BFO 白色 显示
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    spr.drawString("BFO:", 123, 102, 4);
    spr.setTextDatum(MR_DATUM);
    spr.drawString(bfo, 225, 102, 4);
    spr.setTextDatum(MC_DATUM);
  }

  for (int i = 0; i < getStrength(); i++)                                           //信号强度
    if (i < 10) spr.fillRect(250 + (i * 2), leftrect_y+16+space - (i * 1), 2, 4 + (i * 1), 0x3526);  //信号强度
    else spr.fillRect(250 + (i * 2), leftrect_y+16+space - (i * 1), 2, 4 + (i * 1), TFT_RED);
    
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    spr.loadFont(orbitronbold16);
    spr.drawNumber(rssi, 298, leftrect_y+13+space, 2);
    spr.unloadFont();

  spr.fillTriangle(156, 112+8, 160, 122+8, 164, 112+8, TFT_RED);  //频率指示条上方红箭头  //向下移动8
  spr.drawLine(160, 114+8, 160, 170, TFT_RED);                //频率指示红条

  if (muted)  //静音图标和0显示
  {
    spr.pushImage(250, leftrect_y+2+space*3, 18, 18, mute);
    spr.setTextColor(TFT_WHITE, TFT_BLACK);  //TFT_RED 静音数字0改为白色
    spr.loadFont(orbitronbold16);
    spr.drawString("0", 298, leftrect_y+13+space*3);
    spr.unloadFont();
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
  } else  //音量图标和数字显示
  {
    spr.pushImage(250, leftrect_y+2+space*3, 18, 18, speaker);
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    spr.loadFont(orbitronbold16);
    spr.drawNumber(rx.getVolume(), 298, leftrect_y+13+space*3, 2);
    spr.unloadFont();
  }
  
    spr.loadFont(orbitronbold16);
    spr.drawNumber(snr, 298, leftrect_y+13+space*2, 2);
    spr.loadFont(NotoMono16);
    spr.drawString("SNR", 261, leftrect_y+13+space*2, 2);
    spr.unloadFont();
  
  spr.setTextWrap(false);      // 关闭自动换行
  
  spr.loadFont(NotoMono16);
  //spr.setTextDatum(TC_DATUM);
  int temp = (currentFrequency / 10.00) - 20;  //下方频率显示条
  uint16_t lineColor;
  for (int i = 0; i < 40; i++) {
    if (i == 20) lineColor = TFT_RED;
    else lineColor = 0xC638;
    if (!(temp < band[bandIdx].minimumFreq / 10.00 or temp > band[bandIdx].maximumFreq / 10.00)) {
      if ((temp % 10) == 0) {
        spr.drawLine(i * 8, 170, i * 8, 146, lineColor);
        spr.drawLine((i * 8) + 1, 170, (i * 8) + 1, 146, lineColor);
        if (currentMode == FM) spr.drawFloat(temp/10 ,0, i * 8, 133+5, 2);  //temp / 10.0, 1, i * 8, 133, 2 去除频率调小数点 mhz 改为khz
        else if (temp >= 100) spr.drawFloat(temp*10 ,0, i * 8, 133+5, 2);  //temp / 100.0, 3, i * 8, 133, 2
        else { spr.drawNumber(temp * 10, i * 8, 133+5, 2); }
      } else if ((temp % 5) == 0 && (temp % 10) != 0) {
        spr.drawLine(i * 8, 170, i * 8, 154, lineColor);
        spr.drawLine((i * 8) + 1, 170, (i * 8) + 1, 154, lineColor);
        // spr.drawFloat(temp/10.0,1,i*8,144);
      } else {
        spr.drawLine(i * 8, 170, i * 8, 162, lineColor);  //160 150 140
      }
    }

    temp = temp + 1;
  }
  spr.unloadFont();

  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  if (currentMode == FM)  //fm 显示 立体声 单声道
  {
    spr.loadFont(orbitronbold16);
    if (rx.getCurrentPilot()) {
      spr.drawString("FM", 261, leftrect_y+13, 2);  //显示调频立体声
      spr.pushImage(285+4, 22, 19, 19, stereo);
    } else
      spr.drawString("FM", 278, leftrect_y+13, 2);  //显示调频单声道
    spr.unloadFont();
  } else  //除fm显示 模式
  {
    spr.loadFont(orbitronbold16);
    spr.drawString(bandModeDesc[currentMode], 278, 33, 2);
    spr.unloadFont();
  }

  spr.loadFont(HanYiZhongYuanJian16);
  if (cmdseek & !isMenuMode() & !cmdBand) 
  {
    //spr.pushImage(3, 0, 18, 18, seek); 
    spr.pushImage(leftrect_x+50, leftrect_y+2+space*3, 18, 18, seek);
    spr.drawString("搜索", leftrect_x+20, leftrect_y+12+space*3); //搜索模式显示图标 菜单不显示

  }  
  if (!cmdseek & !isMenuMode() & !cmdBand) 
  {
   // spr.pushImage(3, 0, 18, 18, plus);
    spr.pushImage(leftrect_x+50, leftrect_y+2+space*3, 18, 18, plus);
    spr.drawString("频率", leftrect_x+20, leftrect_y+12+space*3); //搜索模式显示图标 菜单不显示

  }
  spr.unloadFont();


  spr.drawString(bufferStationName, 160, 102, 4);  //显示rds获取的电台名称  和搜索模式频率模式 位置冲突 zooc
  spr.setTextColor(TFT_WHITE, TFT_BLACK);

  if(batteryCharging)
  {
    spr.pushImage(XbatPos-20, YbatPos, 12, 12, charge12);
  }
  else
  {
    spr.loadFont(NotoMono16);
    spr.drawString(String(batteryLevel) + "%", XbatPos - 24, YbatPos + 7, 2);  //显示电量百分比  battery.getBatteryChargeLevel()
    spr.unloadFont();
    
    //spr.drawString(String(getBatteryVolts(CONV_FACTOR),3), 20, 110, 4);
  }
  
  if(!lcdoffed)
  {
  batteryMonitor(true);
  
  spr.pushSprite(0, 0); 
  }

}

void cleanBfoRdsInfo() {
  bufferStationName[0] = '\0';
}

void showRDSMsg() {
  rdsMsg[35] = bufferRdsMsg[35] = '\0';
  if (strcmp(bufferRdsMsg, rdsMsg) == 0)
    return;
}

void showRDSStation() {
  if (strcmp(bufferStationName, stationName) == 0) return;
  cleanBfoRdsInfo();
  strcpy(bufferStationName, stationName);
  drawSprite();
}

void showRDSTime() {
  if (strcmp(bufferRdsTime, rdsTime) == 0)
    return;
}

void checkRDS() {
  rx.getRdsStatus();
  if (rx.getRdsReceived()) {
    if (rx.getRdsSync() && rx.getRdsSyncFound()) {
      rdsMsg = rx.getRdsText2A();
      stationName = rx.getRdsText0A();
      rdsTime = rx.getRdsTime();
      // if ( rdsMsg != NULL )   showRDSMsg();
      if (stationName != NULL)
        showRDSStation();
      // if ( rdsTime != NULL ) showRDSTime();
    }
  }
}

void DrawBatteryLevel(int batteryLevel)  //电量图标绘制
{

  int chargeLevel;
  uint16_t batteryLevelColor;

  if (batteryLevel == 1) {
    chargeLevel = 3;
    batteryLevelColor = TFT_RED;
  }
  if (batteryLevel == 2) {
    chargeLevel = 5;
    batteryLevelColor = TFT_YELLOW;
  }
  if (batteryLevel == 3) {
    chargeLevel = 9;
    batteryLevelColor = TFT_YELLOW;
  }
  if (batteryLevel == 4) {
    chargeLevel = 12;
    batteryLevelColor = TFT_GREEN;
  }
  if (batteryLevel == 5) {
    chargeLevel = 15;
    batteryLevelColor = TFT_GREEN;
  }
  if (batteryLevel == 6) {
    chargeLevel = 18;
    batteryLevelColor = TFT_GREEN;
  }
  if (batteryLevel == 7) {
    chargeLevel = 18;
    batteryLevelColor = TFT_BLUE;
  }

  spr.drawRect(XbatPos, YbatPos, Xbatsiz, Ybatsiz, TFT_WHITE);
  spr.fillRect(XbatPos + 1, YbatPos + 1, Xbatsiz - 2, Ybatsiz - 2, TFT_BLACK);
  spr.fillRect(XbatPos + 1, YbatPos + 1, chargeLevel, Ybatsiz - 2, batteryLevelColor);  //电池图标x 18个像素
  spr.fillRect(XbatPos + 20, YbatPos + 2, 2, 5, TFT_WHITE);


  spr.pushSprite(0, 0);
}

void batteryMonitor(bool forced)  //电池百分比 电池电压等级
{
  bool chargechanged = false;
  if(batteryCharging!=chargechanged) chargechanged=true;  //判断充电状态是不是改变
  else chargechanged=false;

  unsigned long currentTime = millis();      // 获取当前时间
  if (currentTime - lastSampleTime >= 30000 | chargechanged )  //每30秒读取一次电量百分比 减少跳动 拔下充电器 立即采样
  {
    batteryLevel = getBatteryChargeLevel(CONV_FACTOR);  // 执行采样操作 setup增加一次 否则开机5s无电量百分比
    lastSampleTime = currentTime;                    // 更新最后一次采样时间
  }

  if (getBatteryVolts(CONV_FACTOR,!lcdoffed) >= MIN_USB_VOLTAGE) {
    if (!batteryCharging or forced) {
      batteryCharging = true;  //充电中 可以增加充电图标
      DrawBatteryLevel(7);
    }
  } else {
    batteryCharging = false;

    if (batteryLevel > 100) {
      currentBatteryLevel = 7;
    } else if (batteryLevel >= 90) {
      currentBatteryLevel = 6;
    } else if (batteryLevel >= 75) {
      currentBatteryLevel = 5;
    } else if (batteryLevel >= 60) {
      currentBatteryLevel = 4;
    } else if (batteryLevel >= 45) {
      currentBatteryLevel = 3;
    } else if (batteryLevel >= 30) {
      currentBatteryLevel = 2;
    } else if (batteryLevel >= 15) {
      currentBatteryLevel = 2;
    } else if (batteryLevel <= 5) {
      currentBatteryLevel = 1;
    }
    
    if (currentBatteryLevel != previousBatteryLevel or forced) {
      DrawBatteryLevel(currentBatteryLevel);
      previousBatteryLevel = currentBatteryLevel;
    }
  }
  yield();
}

bool runEvery(unsigned long &previousMillis, unsigned long interval, void (*task)()) //定义一个函数模板 定时执行
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    task();
    return true; // 返回执行状态
  }
  return false;
}


///////////////////////////////////////////主循环loop//////////////////////////////////////////////////////////////
void loop() {
  
  checkScreenTimeout();                                             //检查操作超时
  if ((currentMode == LSB || currentMode == USB)) cmdseek = false;  // 单边带不支持搜索?

  //runEvery(test1, 5000, []() {rx.volumeDown();}); // 自动捕获外部变量

 if (lcdoffed) //熄屏进入浅睡眠
 {
    tft.writecommand(0x10); //屏幕睡眠
    setCpuFrequencyMhz(80);
    //配置唤醒源，当引脚检测到高电平时唤醒
    pinMode(1, INPUT_PULLUP); // 启用内部上拉电阻
    //pinMode(2, INPUT_PULLUP); // 启用内部上拉电阻
    //(1ULL << BUTTON1_PIN) | (1ULL << BUTTON2_PIN)
    //esp_sleep_enable_ext1_wakeup( (1ULL << GPIO_NUM_1) | (1ULL << GPIO_NUM_2), ESP_EXT1_WAKEUP_ANY_LOW);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_1, LOW);
    esp_light_sleep_start();
 }
 else
 {
    tft.writecommand(0x11); // 退出睡眠
    setCpuFrequencyMhz(240);
 }
  
  event = getButtonEvent(21);

  if (event == 1) {
    if (!lcdoffed)  //亮屏状态执行单机操作
    {
      if (cmdsave) {
        savestation(savenum);
        cmdsave = false;
      } else if (cmdMenu) {
        currentMenuCmd = menuIdx;
        doCurrentMenuCmd();
      } else if (isMenuMode()) {
        disableCommands();
        showStatus();
        showCommandStatus((char *)"VFO ");
      } else if (bfoOn) {
        bfoOn = false;
        showStatus();
      } else {
        cmdBand = !cmdBand;
        menuIdx = BAND;
        currentMenuCmd = menuIdx;
        drawSprite();
      }
    }
    if (lcdoffed) { lcdoff(false); }  //熄屏只执行亮屏操作
    lastActiveTime = millis();
    resetEepromDelay();
  }

  if (event == 2) {
    if (lcdoffed) { lcdoff(false); }
    cmdMenu = !cmdMenu;
    if (cmdMenu) drawSprite();
    lastActiveTime = millis();
    resetEepromDelay();
  }

  if (event == 3) {
    if (lcdoffed) { lcdoff(false); }
    if (readsavednumcount() == 0)  //如果未存储电台 显示提示! zooc
    {
      tft.drawString("no station saved!", 110, 90, 2);  //在rds位置显示调试数据
      delay(1000);
    } else {
      menuIdx = selstation;
      cmdselstation = true;
      doselstation(encoderCount);
    }

    lastActiveTime = millis();
    resetEepromDelay();
  }
  
  if(event == 4)
  {
    CONV_FACTOR = CONV_FACTOR * (4.2/getBatteryVolts(CONV_FACTOR,true)) ;    //重新计算系数CONV_FACTOR

    EEPROM.begin(EEPROM_SIZE);
    EEPROM.write(eeprom_address+10, int((CONV_FACTOR*1000))  >> 8 );              // 存储系数高八位
    EEPROM.write(eeprom_address+11, int((CONV_FACTOR*1000))  &0xFF);              // 存储系数低八位
    EEPROM.commit(); 
    //delay(50); 
    EEPROM.end();
     
    tft.drawString(String(getBatteryVolts(CONV_FACTOR,true),3),0,100,2);  //显示校准后电压
    tft.drawString(String(CONV_FACTOR),40,100,2); //显示校准后系数
    
  }

  if (event == 5)  // 长按
  {
    lcdoff(false);
    if ((currentMode == LSB || currentMode == USB)) cmdseek = false;
    else { cmdseek = !cmdseek; }
    drawSprite();
    lastActiveTime = millis();
    resetEepromDelay();
  }

  if(event==6)
  { 
    disableCommands();
    menuIdx=VOLUME;
    cmdVolume=true;
    doVolume(encoderCount);
    drawSprite();
    lastActiveTime = millis();
    resetEepromDelay();
  }

  // if (checkButtonPress(21, 50, 50, 50))  //digitalRead(21)==LOW
  // {
  //   if (encoderCount != 0)  //转动编码器
  //   {
  //     doVolume(encoderCount);
  //     encoderCount = 0;
  //   }                            //按压旋转 调节音量 在按键函数中 屏蔽按键和长按 checkButtonPress(21, 50, 0, 20)
  // } 
  
  if (encoderCount != 0)  //转动编码器
  {
    if (bfoOn & (currentMode == LSB || currentMode == USB)) {
      currentBFO = (encoderCount == 1) ? (currentBFO + currentBFOStep) : (currentBFO - currentBFOStep);
      rx.setSSBBfo(currentBFO);
      showBFO();
    } else if (cmdMenu) doMenu(encoderCount);
    else if (cmdMode) doMode(encoderCount);
    else if (cmdStep) doStep(encoderCount);
    else if (cmdAgc) doAgc(encoderCount);
    else if (cmdBandwidth) doBandwidth(encoderCount);
    else if (cmdVolume) doVolume(encoderCount);
    else if (cmdSoftMuteMaxAtt) doSoftMute(encoderCount);
    else if (cmdBand) setBand(encoderCount);
    else if (cmdbright) dobright(encoderCount);
    else if (cmdsave) dosavestation(encoderCount);
    else if (cmdautolcdoff) sellcdofftime(encoderCount);
    else if (cmdselstation & !lcdoffed) doselstation(encoderCount);
    else if (cmdseek & !lcdoffed) doSeek(encoderCount);
    else {
      //autostep();  //自动步进 临时测试 未优化

      if (encoderCount == 1 & !lcdoffed) rx.frequencyDown();
      else if (encoderCount == -1 & !lcdoffed) rx.frequencyUp();

      if (currentMode == FM) cleanBfoRdsInfo();
      // Show the current frequency only if it has changed
      currentFrequency = rx.getFrequency();
      showFrequency();
    }

    if (lcdoffed) { lcdoff(false); }
    lastActiveTime = millis();
    encoderCount = 0;
    resetEepromDelay();
    elapsedCommand = millis();
    // }
  }

  if ((millis() - elapsedRSSI) > MIN_ELAPSED_RSSI_TIME * 6)  //显示信号强度 若改变
  {
    rx.getCurrentReceivedSignalQuality();
    snr = rx.getCurrentSNR();
    int aux = rx.getCurrentRSSI();
    if (rssi != aux) {
      rssi = aux;
      showRSSI();
    }
    elapsedRSSI = millis();
  }

  if ((millis() - elapsedCommand) > ELAPSED_COMMAND)  //退出控制命令  读取电台不退出(包括单边带)
  {
    if ((currentMode == LSB || currentMode == USB)) {
      bfoOn = false;
      // showBFO();
      if (cmdselstation)  //如果在读取电台界面不退出菜单 单边带也不推出
      {
        showStatus();
      } else {
        disableCommands();
        showStatus();
      }

    } else if (isMenuMode() or cmdBand) {
      if (cmdselstation)  //如果在读取电台界面不退出菜单
      {
        showStatus();
      } else {
        disableCommands();
        showStatus();
      }
    }
    elapsedCommand = millis();
  }

  if ((millis() - lastRDSCheck) > RDS_CHECK_TIME)  //rds获取条件
  {
    if ((currentMode == FM) and (snr >= 12)) checkRDS();
    lastRDSCheck = millis();
  }

  if (itIsTimeToSave)  //存储用户配置
  {
    if ((millis() - storeTime) > STORE_TIME) {
      saveAllReceiverInformation();
      storeTime = millis();
      itIsTimeToSave = false;
    }
  }

  //delay(5);
}
