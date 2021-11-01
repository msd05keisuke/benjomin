//=====================================================================
//  一定間隔で人感センサの反応をアドバタイズする
//  閾値を設定し閾値以上なら在室になる． 在室=1 空室=0
//  アドバタイズは全チャンネル、間隔は250ms-500ms
//=====================================================================

#include <MsTimer2.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <HTS221.h>
#include <ClosedCube_OPT3001.h>
#include "TBGLib.h"
#include <SoftwareSerial.h>

//=====================================================================
// BLE Local device name
// 長さは必ず6文字
//=====================================================================
String strDeviceName = "BLE_B3";

//=====================================================================
// SLEEP動作の有効無効切り替え
// SLEEP_ENABLE = 0 :無効 SLEEP_ENABLE = 1 :有効
//=====================================================================
#define SLEEP_ENABLE 1

//=====================================================================
// シリアルコンソールへのデバック出力
//    #define DEBUG = 出力あり
//　　// #define DEBUG = 出力なし（コメントアウトする）
//=====================================================================
#define DEBUG

//=====================================================================
// スリープ時間、起動時間、送信間隔の設定
//  SLEEP_INTERVAL :スリープ時間　8秒単位で設定
//  WAKE_INTERVAL　：起動時間（スリープ復帰からスリープまでの時間）1秒単位(アドバタイズ時間）
//=====================================================================
#define SLEEP_INTERVAL 1  // 8s x 1 = 8s
#define WAKE_INTERVAL  1    // 1s

//=====================================================================
// IOピンの名前定義
// 接続するリーフに合わせて定義する
//=====================================================================
// --------------------------------------------
// PD port
//     digital 0: PD0 = PCRX    (HW UART)
//     digital 1: PD1 = PCTX    (HW UART)
//     digital 2: PD2 = INT0#
//     digital 3: PD3 = INT1#
//     digital 4: PD4 = RSV
//     digital 5: PD5 = CN3_D5
//     digital 6: PD6 = DISCN
//     digital 7: PD7 = BLSLP#
// --------------------------------------------
#define PCTX 0
#define PCRX 1
#define INT0 2
#define INT1 3
#define LED 4
#define PIN3_D5 5
#define BLE_RESET_PIN 6
#define BLE_WAKEUP_PIN 7

// --------------------------------------------
// PB port
//     digital 8: PB0 = UART2_RX (software UART)
//     digital 9: PB1 = UART2_TX (software UART)
//     digital 10:PB2 = SS#
//     digital 11:PB3 = MOSI
//     digital 12:PB4 = MISO
//     digital 13:PB5 = SCK (LED)
//                PB6 = XTAL1
//                PB7 = XTAL2
//---------------------------------------------
#define UART2_RX 8
#define UART2_TX 9
#define SS 10
#define MOSI 11
#define MISO 12
#define LED_PIN 13

// --------------------------------------------
// PC port
//     digital 14/ Analog0: PC0 = PIN24_D14
//     digital 15/ Analog1: PC1 = BLETX (software UART)
//     digital 16/ Analog2: PC2 = BLERX (software UART)
//     digital 17/ Analog3: PC3 = PIN27_D17
//     digital 18/ SDA    : PC4 = SDA   (I2C)
//     digital 19/ SCL    : PC5 = SCL   (I2C)
//     RESET              : PC6 = RESET#
//-----------------------------------------------
#define PIN24_D14 14
#define BLETX 15
#define BLERX 16
#define PIN27_D17 17
#define SDA 18
#define SCL 19

//=====================================================================
// プログラム内で使用する定数定義
//
//=====================================================================


//-----------------------------------------------
//人感センサ関連
//-----------------------------------------------
#define I2C_PIR_ADDR   0x65
#define I2C_SEND_BUF_LENGTH 10
#define I2C_RECV_BUF_LENGTH 10

unsigned char i2c_sendBuf[I2C_SEND_BUF_LENGTH];
unsigned char i2c_recvBuf[I2C_RECV_BUF_LENGTH];

float irData=0;






//-----------------------------------------------
// loop() interval
// MsTimer2のタイマー割り込み発生間隔(ms)
//-----------------------------------------------
#define LOOP_INTERVAL 125 // 125ms interval

//-----------------------------------------------
// BLE
//-----------------------------------------------
#define BLE_STATE_STANDBY (0)
#define BLE_STATE_SCANNING (1)
#define BLE_STATE_ADVERTISING (2)
#define BLE_STATE_CONNECTING (3)
#define BLE_STATE_CONNECTED_MASTER (4)
#define BLE_STATE_CONNECTED_SLAVE (5)

//-----------------------------------------------
// Batt ADC ADC081C027
//-----------------------------------------------
#define BATT_ADC_ADDR 0x50

//=====================================================================
// object
//=====================================================================
//-----------------------------------------------
// Sensor
//-----------------------------------------------
Adafruit_LIS3DH accel = Adafruit_LIS3DH();
ClosedCube_OPT3001 light;

//-----------------------------------------------
// BLE
//-----------------------------------------------
SoftwareSerial Serialble(BLERX, BLETX);
BGLib ble112((HardwareSerial *)&Serialble, 0, 0);

//=====================================================================
// プログラムで使用する変数定義
//
//=====================================================================
//=====================================================================
// RAM data
//=====================================================================
//---------------------------
// loop counter
//---------------------------
uint8_t iLoop1s = 0;
uint8_t iSleepCounter = 0;
uint8_t iSendCounter = 0;

//---------------------------
// event
//---------------------------
bool eventSleepCheck = false;

//---------------------------
// interval Timer2 interrupt
//---------------------------
volatile bool bInterval = false;



//---------------------------
// BLE
//---------------------------
bool bBLEconnect = false;
bool bBLEsendData = false;
int8_t bleSendCount = 0;

volatile uint8_t ble_state = BLE_STATE_STANDBY;
volatile uint8_t ble_encrypted = 0;  // 0 = not encrypted, otherwise = encrypted
volatile uint8_t ble_bonding = 0xFF; // 0xFF = no bonding, otherwise = bonding handle

//---------------------------
// Sleep, Watchdog Timer
//---------------------------
volatile bool bSleep = false;

volatile int countWDT = 0;
volatile int wakeupWDT = SLEEP_INTERVAL;



//=====================================================================
// setup
//=====================================================================
//-----------------------------------------------
// port
//-----------------------------------------------
//=====================================================================
// IOピンの入出力設定
// 接続するリーフに合わせて設定する
//=====================================================================
void setupPort()
{
  //---------------------
  // PD port
  //---------------------
  // PD0 : digital 0 = RX
  // PD1 : digital 1 = TX

  pinMode(INT0, INPUT); // PD2 : digital 2 = BLE interrupt
  pinMode(INT1, INPUT); // PD3 : digital 3 = sensor interrupt

  pinMode(LED, OUTPUT); // PD4 : digital 4 = LED
  digitalWrite(LED, LOW);

  pinMode(PIN3_D5, INPUT); // PD5 : digital 5 = not in use

  pinMode(BLE_RESET_PIN, OUTPUT); // PD6 : digital 6 = BLE reset active-low
  digitalWrite(BLE_RESET_PIN, LOW);

  pinMode(BLE_WAKEUP_PIN, OUTPUT); // PD7 : digital 7 = BLE sleep
  digitalWrite(BLE_WAKEUP_PIN, HIGH);

  //---------------------
  // PB port
  //---------------------
  pinMode(UART2_RX, INPUT); // PB0 : digital 8 = not in use
  pinMode(UART2_TX, INPUT); // PB1 : digital 9 = not in use

  pinMode(SS, INPUT);   // PB2 : digital 10 = not in use
  pinMode(MOSI, INPUT); // PB3 : digital 11 = not in use
  pinMode(MISO, INPUT); // PB4 : digital 12 = not in use

  pinMode(LED_PIN, OUTPUT); // PB5 : digital 13 =LED on 8bit-Dev. Leaf
  digitalWrite(LED_PIN, LOW);

  //---------------------
  // PC port
  //---------------------
  pinMode(PIN24_D14, INPUT); // PC0 : digital 14 = not in use

  // PC1 : digital 15 = BLETX
  // PC2 : digital 16 = BLERX

  pinMode(PIN27_D17, INPUT); // PC3 : digital 17  = not in use

  // PC4 : digital 18 = I2C SDA
  // PC5 : digital 19 = I2C SCL
}
//=====================================================================
// 割り込み処理初期設定
//
//=====================================================================
//-----------------------------------------------
// external interrupt
// 外部割り込み設定
//-----------------------------------------------
void setupExtInt()
{
  attachInterrupt(0, intExtInt0, FALLING); // BLE    INT0# = enabled
  detachInterrupt(1);                      // sensor INT1# = disabled
}

//-----------------------------------------------
// timer2 interrupt (interval=125ms, int=overflow)
// メインループのタイマー割り込み設定
//-----------------------------------------------
void setupTC2Int()
{
  MsTimer2::set(LOOP_INTERVAL, intTimer2);
}

//=====================================================================
// 各デバイスの初期設定
//
//=====================================================================
//-----------------------------------------------
// sensor
//-----------------------------------------------


void setupSensor()
{
  // セットアップ！！！！！！！！！！！！！！！！！！！！！！！！！！！
  //人感センサ設定
  i2c_write_byte(I2C_PIR_ADDR, 0x20, 0xFF); //CNTL1  Resrt
  i2c_write_byte(I2C_PIR_ADDR, 0x2A, 0xF2); //CNTL11 人感アルゴリズム有効/割り込み出力有効
  i2c_write_byte(I2C_PIR_ADDR, 0x25, 0x0F); //CNTL6  センサゲイン205%(最大)
  i2c_write_byte(I2C_PIR_ADDR, 0x2B, 0xFF); //CNTL12 Mode=1 start Meas(連続測定モード)
  delay(1000);


}

//-----------------------------------------------
// BLE
//-----------------------------------------------
void setupBLE()
{
  String stWork;

  // set up internal status handlers (these are technically optional)
  ble112.onBusy = onBusy;
  ble112.onIdle = onIdle;
  ble112.onTimeout = onTimeout;
  // ONLY enable these if you are using the <wakeup_pin> parameter in your firmware's hardware.xml file
  // BLE module must be woken up before sending any UART data

  // set up BGLib response handlers (called almost immediately after sending commands)
  // (these are also technicaly optional)

  // set up BGLib event handlers
  /* [gatt_server] */
  ble112.ble_evt_gatt_server_attribute_value = my_evt_gatt_server_attribute_value; /* [BGLib] */
  /* [le_connection] */
  ble112.ble_evt_le_connection_opend = my_evt_le_connection_opend;   /* [BGLib] */
  ble112.ble_evt_le_connection_closed = my_evt_le_connection_closed; /* [BGLib] */
  /* [system] */
  ble112.ble_evt_system_boot = my_evt_system_boot; /* [BGLib] */

  ble112.ble_evt_system_awake = my_evt_system_awake;
  ble112.ble_rsp_system_get_bt_address = my_rsp_system_get_bt_address;
  /*  */

  Serialble.begin(9600);

  /* interval_min : 250ms( = 400 x 0.625ms ) */
  /* interval_max : 500ms( = 800 x 0.625ms ) */
  ble112.ble_cmd_le_gap_set_adv_parameters(400, 800, 7); /* [BGLIB] <interval_min> <interval_max> <channel_map> */
  while (ble112.checkActivity(1000))
    ; /* [BGLIB] 受信チェック */
}

//-----------------------------------------------
// アドバタイズするデータの設定
//-----------------------------------------------
void StartAdvData()
{
  uint8 dataLen;
  uint8 stLen;
  uint8 adv_data[25];  // advertising data (max 25bytes)
  uint8 index = 0;

  short int flg = 0; // １なら正　０なら負 
  short int data = 0;
  short int result = 0; //結果



  //-------------------------
  // 符号判定
  //-------------------------
  if(irData >= 0)
  {
    flg = 1;
    data = irData;
    
  }
  else
  {
    data = (short int)irData*(short int)(-1);
    // Serial.println((short int)irData*(short int)(-1));
  }

  //調整桁
  float tmp = data*0.01;
  // Serial.println(tmp);

  
  result = (short int)(tmp * 256);

  // Serial.println(result);
  // Serial.println(flg);

  
 
  //-------------------------
  // Advertising Packet
  //-------------------------
  // AD Structure 1 (Flags)
  adv_data[index++] = 0x02;                       // field length
  adv_data[index++] = BGLIB_GAP_AD_TYPE_FLAGS;    // AD Type (Flags)
  adv_data[index++] = (1 << 1) | (1 << 2);        // LE General Discover Mode | BR/EDR Not Supported

  // AD Structure 2 (Complete Local Name)
  adv_data[index++] = strDeviceName.length() + 1;  // field length
  adv_data[index++] = BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE;  // AD Type (Complete Local Name)
  for (uint8 i = 0; i < strDeviceName.length(); i++){
    adv_data[index++] = strDeviceName.charAt(i);  // Local Name
  }

  // AD Structure 3 (Manufacturer Specific Data)
  adv_data[index++] = 4;           // field lengh
  adv_data[index++] = 0xff;         // AD Type (Manufacturer Specific Data)
  adv_data[index++] = flg;
  adv_data[index++] = (result >> 8) & 0xFF; // data[1]     
  adv_data[index++] = result & 0xFF; // data[2]
  // adv_data[index++] = 0;
  // adv_data[index++] = 0;

  //アドバタイズデータを登録
  stLen = index;
  ble112.ble_cmd_le_gap_set_adv_data(SCAN_RSP_ADVERTISING_PACKETS, stLen, adv_data); //SCAN_RSP_ADVERTISING_PACKETS
  while (ble112.checkActivity(1000));  // 受信チェック

  /* start */
  // index = 0  LE_GAP_SCANNABLE_NON_CONNECTABLE / LE_GAP_UNDIRECTED_CONNECTABLE
  ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_USER_DATA, LE_GAP_SCANNABLE_NON_CONNECTABLE);
  while (ble112.checkActivity(1000));  // 受信チェック
}

//=====================================================================
// 割り込み処理
//
//=====================================================================
//=====================================================================
// interrupt
//=====================================================================
//----------------------------------------------
// Timer2 INT
// タイマー割り込み関数
//----------------------------------------------
void intTimer2()
{
  bInterval = true;
}

//---------------------------------------------
// Watchdog Timer INT
// WDT割り込み関数
//---------------------------------------------
ISR(WDT_vect)
{
  wdt_disable();

  if (bSleep == true)
  {
    countWDT += 1;

    if (countWDT >= wakeupWDT)
    {
      countWDT = 0;
      bSleep = false;
    }
  }
}

//----------------------------------------------
// INT0
// INT0割り込み関数
//----------------------------------------------
void intExtInt0()
{
  bSleep = false;
}

//----------------------------------------------
// INT1
// INT1割り込み関数
//----------------------------------------------
void intExtInt1()
{
  bSleep = false;
}

//====================================================================
// functions
//====================================================================
//--------------------------------------------------------------------
// counter /event
//--------------------------------------------------------------------
//-----------------------------------------
// main loop
// メインループのループ回数をカウントし
// 1秒間隔でセンサーデータの取得とBLEの送信をONにする
// 4秒間隔でスリープ確認をONにする
//-----------------------------------------
void loopCounter()
{
  iLoop1s += 1;

  // Serial.println(iLoop1s);

  //--------------------
  // 1s period
  //--------------------
  if (iLoop1s >= 8)
  { // 125ms x 1 = 125ms

    iLoop1s = 0;
    iSleepCounter += 1;
    //-----------------------
    // WAKE TIME period
    //-----------------------
    if (iSleepCounter >= WAKE_INTERVAL)
    {

      iSleepCounter = 0;
      if (SLEEP_ENABLE)
      {
        eventSleepCheck = true;
      }
    }
  }
}


//--------------------------------------------------------------------
// センサの値を取得
//--------------------------------------------------------------------
void getSensor()
{
  //-------------------------
  // 人感センサーのデータ取得
  //-------------------------
  // Clear buffer
  clearI2CReadbuf();
  // Register read
  i2c_read(I2C_PIR_ADDR, 0x04, 6, i2c_recvBuf);

  irData = clacIR();
 

#ifdef DEBUG
  Serial.println("");
  Serial.println("--- sensor data ---");
  Serial.println("  IR     = " + String(irData));
  Serial.println("");
#endif
}

//--------------------------------------------------------------------
// sleep
//--------------------------------------------------------------------
//-----------------------------------------
// main loop
// スリープ移行要求があった場合、センサーリーフ、BLEリーフをSLEEPさせて
// WDTをセットしマイコンリーフをスリープさせる
//-----------------------------------------
void loopSleep()
{
  if (eventSleepCheck == true)
  {
    eventSleepCheck = false;

    bSleep = true;
    countWDT = 0;

#ifdef DEBUG
    Serial.print(F("  >>> Go to sleep :  wakeup after = "));
    Serial.print(wakeupWDT, DEC);
    Serial.println(" x 8s  >>>");
    Serial.flush();
#endif

    //-----------------------
    // sensor sleep
    //-----------------------
    sleepSensor();
#ifdef DEBUG
    Serial.println(F("Sleep Sensor"));
#endif
    //-----------------------
    // BLE sleep
    //-----------------------
    sleepBLE();
#ifdef DEBUG
    Serial.println(F("Sleep BLE"));
#endif

    //-----------------------
    // ATMega328 sleep
    //-----------------------
#ifdef DEBUG
    Serial.println(F("Sleep AVR"));
    Serial.flush();
#endif
    while (bSleep == true)
    {
      wdt_start();
      
      sleep();
    }
    delay(100);
#ifdef DEBUG
    Serial.println(F("Wakeup AVR"));
#endif
    //-----------------------
    // BLE wakeup
    //-----------------------
    wakeupBLE();
#ifdef DEBUG
    Serial.println(F("Wakeup BLE"));
#endif
    //-----------------------
    // sensor wakeup
    //-----------------------
    wakeupSensor();
#ifdef DEBUG
    Serial.println(F("Wakeup Senser"));
#endif

#ifdef DEBUG
    Serial.println(F("  <<< Wake up <<<"));
#endif
    //-----------------------
    // start ADV
    //-----------------------
    delay(100);
    getSensor();

#ifdef DEBUG
    Serial.println(F("Start advertise"));
#endif
  }
}
//-----------------------------------------
// SLEEP
//-----------------------------------------
void sleep()
{
  ADCSRA &= ~(1 << ADEN); //ADC停止
  //PRR = 0x05;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //SET SLEEP MODE
  sleep_enable();                      // SLEEP ENABLE

  // BOD停止
  MCUCR |= (1 << BODSE) | (1 << BODS);           // MCUCRのBODSとBODSEに1をセット
  MCUCR = (MCUCR & ~(1 << BODSE)) | (1 << BODS); // すぐに（4クロック以内）BODSSEを0, BODSを1に設定
  asm("sleep");                                  // 3クロック以内にスリープ
  sleep_disable();                               // SLEEP DISABLE
}
//-----------------------------------------
// WDT
//-----------------------------------------
void wdt_start()
{
  // watchdog timer reset
  wdt_reset();

  //disable interruput
  cli();
  //clear WatchDog system Reset Flag(WDRF)
  MCUSR &= ~(1 << WDRF);
  // WDT変更許可
  // WDCEとWDE同時セットで変更許可
  WDTCSR |= 1 << WDCE | 1 << WDE;
  //WDT設定
  // WDE=0,WDIE=1 :WDT overflowで割り込み
  // WDP3=0,WDP2=0,WDP1=0,WDP0=1: 1s            間隔を変更するのはこちら！！！！！！！！！！！！！！！！！！！！！
  
  WDTCSR = 1 << WDIE | 0 << WDE | 0 << WDP3 | 0 << WDP2 | 0 << WDP1 | 1 << WDP0;

  
  //enable interruput
  sei();
}
//-----------------------------------------
// sleep sensor
// センサーリーフをスリープさせる
//-----------------------------------------
void sleepSensor()
{
  
  
}

//-----------------------------------------
// wakeup sensor
// センサーリーフをスリープから復帰させる
//-----------------------------------------
void wakeupSensor()
{
  
}

//---------------------------------------
// sleep BLE
// BLE リーフをスリープさせる
//---------------------------------------
void sleepBLE()
{
  ble112.ble_cmd_le_gap_stop_advertising(0);
  Serial.println("gap stop advertising");
  while (ble112.checkActivity())
    ;
  delay(1000);
  ble112.ble_cmd_system_halt(1);
   Serial.println("before while2 in sleepBLE");
   while (ble112.checkActivity())
    ;
  digitalWrite(BLE_WAKEUP_PIN, LOW);
  delay(500);
  Serialble.end();
}

//---------------------------------------
// wakeup BLE
// BLEリーフをスリープから復帰させる
//---------------------------------------
void wakeupBLE()
{
  uint8_t *last;
  Serialble.begin(9600);
  delay(100);
  digitalWrite(BLE_WAKEUP_PIN, HIGH);
  delay(100);

  while (1)
  {
    ble112.checkActivity(); /* 受信チェック */
    last = ble112.getLastEvent();
    if (last[0] == 0x01 && last[1] == 0x04)
      break; /* [evt_system_awake] [2]と[3] */
  }

  ble112.ble_cmd_le_gap_set_adv_parameters(400, 800, 7); /* [BGLIB] <interval_min> <interval_max> <channel_map> */
  while (ble112.checkActivity(1000))
    ; /* [BGLIB] 受信チェック */
}

//====================================================================
// setup
//====================================================================
void setup()
{

  //WDT disable
  wdt_disable();
  delay(10);

  Serial.begin(115200);
  Wire.begin(); // I2C 100KHz
#ifdef DEBUG
  Serial.println(F("========================================="));
  Serial.println(F("setup start"));
#endif

  setupPort();
  delay(10);

  noInterrupts();
  //setupExtInt();
  setupTC2Int();
  interrupts();

  setupSensor();
  setupBLE();
#ifdef DEBUG
  Serial.println(F("setup end"));
#endif
  ble112.ble_cmd_system_get_bt_address();
  while (ble112.checkActivity(1000))
    ;
  delay(1000);

  MsTimer2::start(); // Timer2 inverval start

#ifdef DEBUG
  Serial.println(F(""));
  Serial.println(F("loop start"));
  Serial.println(F(""));
#endif
  getSensor();

}

//====================================================================
// loop
//====================================================================
void loop()
{

  //-----------------------------------------------------
  // Timer2 interval　125ms で1回ループ
  //-----------------------------------------------------
  if (bInterval == true)
  {
    bInterval = false;
    //--------------------------------------------
    //--------------------------------------------
    // loop counter
    //--------------------------------------------
    loopCounter();
    //--------------------------------------------
    // sleep/resume
    //--------------------------------------------
    loopSleep();

    StartAdvData();
  }
}

// ================================================================
// INTERNAL BGLIB CLASS CALLBACK FUNCTIONS
// ================================================================

// called when the module begins sending a command
void onBusy()
{
  // turn LED on when we're busy
  //digitalWrite( LED_PIN, HIGH );
}

// called when the module receives a complete response or "system_boot" event
void onIdle()
{
  // turn LED off when we're no longer busy
  //digitalWrite( LED_PIN, LOW );
}

// called when the parser does not read the expected response in the specified time limit
void onTimeout()
{
  // reset module (might be a bit drastic for a timeout condition though)
  //digitalWrite( BLE_RESET_PIN, LOW );
  //delay(5);                           // wait 5ms
  //digitalWrite( BLE_RESET_PIN, HIGH );
  ble_state = BLE_STATE_STANDBY;

  // clear "encrypted" and "bonding" info
  ble_encrypted = 0;
  ble_bonding = 0xFF;
  bSleep = false;
}

// called immediately before beginning UART TX of a command
void onBeforeTXCommand()
{
}

// called immediately after finishing UART TX
void onTXCommandComplete()
{
  // allow module to return to sleep (assuming here that digital pin 5 is connected to the BLE wake-up pin)
}
/*  */

void my_evt_gatt_server_attribute_value(const struct ble_msg_gatt_server_attribute_value_evt_t *msg)
{
  uint16 attribute = (uint16)msg->attribute;
  uint16 offset = 0;
  uint8 value_len = msg->value.len;

  uint8 value_data[20];
  String rcv_data;
  rcv_data = "";

#ifdef DEBUG
  Serial.print(F("###\tgatt_server_attribute_value: { "));
  Serial.print(F("connection: "));
  Serial.print(msg->connection, HEX);
  Serial.print(F(", attribute: "));
  Serial.print((uint16_t)msg->attribute, HEX);
  Serial.print(F(", att_opcode: "));
  Serial.print(msg->att_opcode, HEX);
  Serial.println(F(" }"));
#endif

  for (uint8_t i = 0; i < value_len; i++)
  {
    rcv_data += (char)(msg->value.data[i]);
  }
}
/*  */

void my_evt_le_connection_opend(const ble_msg_le_connection_opend_evt_t *msg)
{
#ifdef DEBUG
  Serial.print(F("###\tconnection_opend: { "));
  Serial.print(F("address: "));
  // this is a "bd_addr" data type, which is a 6-byte uint8_t array
  for (uint8_t i = 0; i < 6; i++)
  {
    if (msg->address.addr[i] < 16)
      Serial.write('0');
    Serial.print(msg->address.addr[i], HEX);
  }
  Serial.println(" }");
#endif

  /*  */
  ble_state = BLE_STATE_CONNECTED_SLAVE;
  bSleep = false;
}
/*  */
void my_evt_le_connection_closed(const struct ble_msg_le_connection_closed_evt_t *msg)
{
#ifdef DEBUG
  Serial.print(F("###\tconnection_closed: { "));
  Serial.print(F("reason: "));
  Serial.print((uint16_t)msg->reason, HEX);
  Serial.print(F(", connection: "));
  Serial.print(msg->connection, HEX);
  Serial.println(F(" }"));
#endif

  // after disconnection, resume advertising as discoverable/connectable (with user-defined advertisement data)
  //ble112.ble_cmd_le_gap_set_mode( LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE );
  //ble112.ble_cmd_le_gap_start_advertising(1, LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE);
  //ble112.ble_cmd_le_gap_start_advertising( 0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE );    // index = 0
  //while (ble112.checkActivity(1000));

  // set state to ADVERTISING
  ble_state = BLE_STATE_STANDBY;

  // clear "encrypted" and "bonding" info
  ble_encrypted = 0;
  ble_bonding = 0xFF;
  /*  */
  bSleep = false;
}
/*  */

void my_evt_system_boot(const ble_msg_system_boot_evt_t *msg)
{
  ble_state = BLE_STATE_STANDBY;
}
void my_evt_system_awake(const ble_msg_system_boot_evt_t *msg)
{
#ifdef DEBUG
  Serial.print("###\tsystem_awalk: { ");
  Serial.println(" }");
#endif

  ble112.ble_cmd_system_halt(0);
  while (ble112.checkActivity(1000))
    ;
}
void my_rsp_system_get_bt_address(const struct ble_msg_system_get_bt_address_rsp_t *msg)
{
#ifdef DEBUG
  Serial.print("###\tsystem_get_bt_address: { ");
  Serial.print("address: ");
  for (int i = 0; i < 6; i++)
  {
    Serial.print(msg->address.addr[i], HEX);
  }
  Serial.println(" }");
#endif
  unsigned short addr = 0;
  char cAddr[30];
  addr = msg->address.addr[0] + (msg->address.addr[1] * 0x100);
  sprintf(cAddr, "Device name is Leaf_A_#%05d ", addr);
  Serial.println(cAddr);
}










double clacIR()
{
  double ret;
  unsigned short val = (unsigned short)((i2c_recvBuf[2] << 8) |  i2c_recvBuf[1]);
  if ( (val & 0x8000) == 0x8000)
  {
    val = ~val + 1;
    ret = (double)(val *   0.4578 ) * -1;
  }
  else
  {
    ret = (double)(val *  0.4578 );
  }
  return ret;
}


/**********************************************
* I2C Write 1 byte to the slave device
**********************************************/
void i2c_write_byte(int device_address, int reg_address, int write_data){
  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.write(write_data);
  Wire.endTransmission();
}


/**********************************************
* I2C Read 1 byte from the slave device
**********************************************/
unsigned char i2c_read_byte(int device_address, int reg_address){

  int read_data = 0;

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.endTransmission(false);

  Wire.requestFrom(device_address, 1);
  read_data = Wire.read();

  return read_data;
}


/**********************************************
* I2C Write multiple bytes to the slave device
**********************************************/
void i2c_write(int device_address, int reg_address, int lengrh, unsigned char* write_byte){

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  for (int i = 0; i < lengrh; i++){
    Wire.write(write_byte[i]);
  }
  Wire.endTransmission();
}


/**********************************************
* I2C Read multiple bytes from the slave device
**********************************************/
void i2c_read(int device_address, int reg_address, int lengrh, unsigned char* read_byte){

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.endTransmission(false);

  Wire.requestFrom(device_address, lengrh);
  for (int i = 0; i < lengrh; i++){
    read_byte[i] = Wire.read();
  }
}


/**********************************************
* I2C Receive buffer clear
**********************************************/
void clearI2CReadbuf(){
  memset(&i2c_recvBuf[0], 0x00, sizeof(i2c_recvBuf));
}
