// The below sketch and implementation is based on:


/*********************************************************************************
 *  MIT License
 *  
 *  Copyright (c) 2020-2022 Gregg E. Berman
 *  
 *  https://github.com/HomeSpan/HomeSpan
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *  
 ********************************************************************************/
 

// SETTINGS:

// #define DEVICE_ID       0    // ESP32-S2 - DONT USE IT - NOT ENOUGH MEMORY!
#define DEVICE_ID       1    // ESP32-S  - MAIN UNIT
// #define DEVICE_ID       2    // ESP32-S3 - TEST

// #define DEBUG
// #define DEBUG_XTASKS

// #define OTA_ACTIVE                // OTA webserver - is it needed if remote update is working and no remote access anyway?

// libs
#include <Arduino.h>
#include "devices_config.h"
#include <esp_wifi.h>
#include "HomeSpan.h"
#include "passwords.h"
#include <Wire.h>
#ifdef OTA_ACTIVE
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <AsyncElegantOTA.h>
  AsyncWebServer server(8080);
  char ota_user[32];
  char ota_password[64];
  // #warning "OTA_ACTIVE defined"
#else
  #warning "OTA_ACTIVE NOT defined"
#endif

// Firmware update
#include <HTTPClient.h>
#include <Update.h>
#if   (BOARD_TYPE == 1)
  #define FW_BIN_FILE "bridge.esp32.bin"
#elif (BOARD_TYPE == 2)
  #define FW_BIN_FILE "bridge.esp32s2.bin"
#elif (BOARD_TYPE == 3)
  #define FW_BIN_FILE "bridge.esp32s3.bin"
#elif (BOARD_TYPE == 4)
  #define FW_BIN_FILE "bridge.esp32c3.bin"
#else
  #error "FW update defined only for ESP32, ESP32-S2 and ESP32-S3 boards"
#endif

HTTPClient firmware_update_client;
int fw_totalLength = 0;
int fw_currentLength = 0;
bool perform_update_firmware=false;
int update_progress=0;
int old_update_progress=0;
bool blink_led_status=false;
bool fw_update = false;
// Firmware update END

// MAX17048 - battery fuel gauge, I2C
#if (USE_MAX17048 == 1)
  #include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
  SFE_MAX1704X lipo(MAX1704X_MAX17048);
#endif


// DEVICES
// sensors fake MAC addresses - must be the same on sensor devices!
#define DEVICE_ID_1               "1A:01:01:01:01:01" // C3
#define DEVICE_ID_1_NAME          "box 1 sensors"
#define DEVICE_ID_2               "1A:01:01:01:01:02" // S2 - without the box - development, office
#define DEVICE_ID_2_NAME          "box 2 sensors"
#define DEVICE_ID_3               "1A:01:01:01:01:03" // C3
#define DEVICE_ID_3_NAME          "box 3 sensors"
// test device
// #define DEVICE_ID_4               "1A:01:01:01:01:04" // S2
// #define DEVICE_ID_4_NAME          "box 4 sensors"

// DEVICES END

// BRIDGE firmware:
#define BRIDGE_FW                 "1.0.6"     // only numbers here, major: 0-99, minor: 0-9, patch: 0-9 - if letters used they will be ignored on HomeKit 

// folder on web with firmware files
#define CLIENT                    "001-fv"

// macros
#define MANUFACTURER              "PAPIO LTD"
#define NTP_SERVER                "pool.ntp.org"
#define NTP_SERVER_TIMEOUT_S      30
#define LOG_LEVEL                 0
#define UPDATE_TIMEOUT_S          1800   // 1800 = 30min, after this time Bridge reports failure of SENSOR (REMOTE) DEVICE 
#define MIN_ALLOWED_HEAP_SIZE     20000  // restart ESP if below, checked in check_volts()
#define MAX_WEBLOG_ENTRIES        180    // 3 sensors * 12 time per hour * 5 hours = 180, each entry is around 200 bytes

//use one of them below: STATUS LED or ERROR LED  - blinks when new message comes from DEVICES
// #define BLINK_STATUS_LED_ON_RECEIVED_DATA   
#define BLINK_ERROR_LED_ON_RECEIVED_DATA    

#define BATTERY_INTERVAL_S        180   // in seconds, how often to update battery status and charging status of the bridge on HomeKit (battery/charging is measured every second)
#define LOW_BATTERY_THRESHOLD     30  // in % to start complaining (low battery status = 1)
#define IDENTIFY_BLINKS           10  // number of blinks when identify is called (bridge only)

#if (BOARD_TYPE == 2)
  #error "ESP32-S2 does NOT have enough memory for FW update!!!"
  #error "choose anothe board or remove this error to continue" 
#endif

// END of SETTING

typedef struct struct_message          // 36 bytes
{
  float md_temp;
  float md_hum;
  float md_light;
  uint8_t md_bat;
  char md_version[20];
  uint8_t md_mcu_model;
  uint8_t md_charging;
} struct_message;

struct_message myData;
// aux variables
float temperature_value;
float humidity_value;
float light_value;
uint8_t low_bat           = 1; 
uint8_t bat_value_pct;
char md_version_value[20];
uint8_t md_mcu_model_value= 0;
const char models[10][15]        = {"other", "ESP32", "ESP32-S2", "ESP32-S3", "ESP32-C3"}; //    // [number of models][length of string] - used only in Serial not on HomeKit
uint8_t md_charging_value = 0;


//change_mac variables used also in make_fw_version() so must be global
char mac_org_char[18];
uint8_t mac_org[6];
char mac_new_char[18];
uint8_t mac_new[6];

// for data received:
bool temperature_received = false;
bool humidity_received    = false;
bool light_received       = false;
bool bat_received         = false;
bool charging_received    = false;

bool device_timeout = false;      // used to send fault=1 to HomeKit for each sensor if info is not comming for UPDATE_TIMEOUT_S time

// for tasks
BaseType_t xReturned_check_volts;
TaskHandle_t check_volts_handle = NULL;
BaseType_t xReturned_led_blink;
TaskHandle_t led_blink_handle = NULL;

BaseType_t xReturned_check_charging;
TaskHandle_t check_charging_handle = NULL;

// aux for blinking as pointers was segfaulting 
bool blinking = false;

float volts, bat_pct; 
bool check_battery = false;
uint8_t charging_int = 0;          // 0-NC,  1-ON, 2-FULL,   // 3-off ??
const char charging_states[3][5] = {"NC", "ON", "FULL"};  // "OFF"};

// declarations
void led_blink(void *pvParams);
void check_volts(void*z);
void do_restart_esp(const char *message);
void change_mac();
void make_serial_number(char *org_mac, char *new_mac, char *serial_number);
void make_fw_version(const char *fw_numeric, char *fw_with_date_time);

// fw update
void do_update();
void updateFirmware(uint8_t *data, size_t len);
int update_firmware_prepare();


// blinking in rtos
void led_blink(void *pvParams) 
{
  // inspecting stack size for tasks
  #ifdef DEBUG_XTASKS
    UBaseType_t uxHighWaterMark;
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  #endif

  #if defined (ERROR_RED_LED_GPIO) // and defined (ERROR_RED_LED_GPIO)
    while (1) {
        digitalWrite(ERROR_RED_LED_GPIO,HIGH);
        vTaskDelay(50/portTICK_RATE_MS);
        digitalWrite(ERROR_RED_LED_GPIO,LOW);
        vTaskDelay(50/portTICK_RATE_MS);

        #ifdef DEBUG_XTASKS
          uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
          LOG1("[%s]: led_blink uxHighWaterMark=%d\n",__func__,uxHighWaterMark);
        #endif
    }
  #endif
}
// blinking in rtos END

// restart
void do_restart_esp(const char *message)
{
  #if defined (ERROR_RED_LED_GPIO) 
    digitalWrite(ERROR_RED_LED_GPIO,LOW);
  #endif
  #if defined (STATUS_LED_GPIO) 
    digitalWrite(STATUS_LED_GPIO,LOW);
  #endif
  LOG0("\n[%s]: %s\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n          RESTARTING ESP!\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n",__func__,message);
  ESP.restart();
}

void check_volts(void*z)
{
  unsigned long free_mem;

  // inspecting stack size for tasks
  #ifdef DEBUG_XTASKS
    UBaseType_t uxHighWaterMark;
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  #endif

    while(1)
      {
        // check free memory
        free_mem = ESP.getFreeHeap();
        LOG1("[%s]: Free heap=%u bytes\n",__func__,free_mem);
        if (free_mem < MIN_ALLOWED_HEAP_SIZE) do_restart_esp("Free heap too small!");

        // check battery
        #if (USE_MAX17048 == 1)
          volts   = lipo.getVoltage();
          bat_pct = lipo.getSOC();
          if (bat_pct>100) bat_pct=100; // we don't need crazy % here
          if (bat_pct<0)   bat_pct=0;

          LOG2("[%s]: Bridge volts: %0.2f, battery percent=%0.2f%%\n",__func__,volts,bat_pct);
          if (bat_pct < LOW_BATTERY_THRESHOLD)
          {
            #ifdef DEBUG
              Serial.printf("[%s]: Volts too low: %0.2f, battery percent=%0.2f%%\n",__func__,volts,bat_pct);
            #endif
            // blink LEDs in task, if not yet blinking
            if (!blinking)
            {
              xReturned_led_blink = xTaskCreate(led_blink, "led_blink", 1800, NULL, 1, &led_blink_handle);
              if( xReturned_led_blink != pdPASS )
              {
                Serial.printf("[%s]: CANNOT create led_blink task\n",__func__);
              } else 
              {
                #ifdef DEBUG
                  Serial.printf("[%s]: Task led_blink created\n",__func__);
                #endif
                blinking = true;
              }
            } else 
            // blink LEDs already blinking
            {
              #ifdef DEBUG
                Serial.printf("[%s]: Task led_blink ALREADY created\n",__func__);
              #endif
            }
          } else
          {
            #ifdef DEBUG
              Serial.printf("[%s]: Volts OK: %0.2f, battery percent=%0.2f%%\n",__func__,volts,bat_pct);
            #endif
            if (blinking)
            {
              #ifdef DEBUG
                Serial.printf("[%s]: Disabling blinking LEDs\n",__func__);
                Serial.printf("[%s]: DELETING TASK\n",__func__);
              #endif
              vTaskDelete( led_blink_handle );
              blinking = false;
            }
          }
        #endif
        // check battery END

        // check charging
        #if defined(CHARGING_GPIO) and defined(POWER_GPIO)
        /*
          - both GPIO must be PULLUP as LOW is active from TP4056
          - LEDs on TP4056 are NOT needed if PULL_UP both GPIO
          
          #define POWER_GPIO                38  // GREEN, STDB PIN6 on TP4056, LOW on CHARGED (LED ON),       comment out if not in use - don't use "0" here
          #define CHARGING_GPIO             39  // RED,   CHRG PIN7 on TP4056, LOW during CHARGING (LED ON),  comment out if not in use - don't use "0" here
          
          truth table:
          NC:               POWER_GPIO HIGH (PULLUP)    CHARGING_GPIO HIGH (PULLUP)
          CHARGING:         POWER_GPIO HIGH (PULLUP)    CHARGING_GPIO LOW     
          FULL (CHARGED):   POWER_GPIO LOW              CHARGING_GPIO HIGH (PULLUP)
          OFF (DISABLED):   POWER_GPIO HIGH (PULLUP)    CHARGING_GPIO HIGH (PULLUP) ???? - not checked yet

          */

          uint8_t power_gpio_state    = digitalRead(POWER_GPIO);
          uint8_t charging_gpio_state = digitalRead(CHARGING_GPIO);

          if ((power_gpio_state)   and (charging_gpio_state))   charging_int = 0; // NC
          if ((power_gpio_state)   and (!charging_gpio_state))  charging_int = 1; // ON/CHARGING
          if ((!power_gpio_state)  and (charging_gpio_state))   charging_int = 2; // FULL/CHARGED
          // if ((digitalRead(POWER_GPIO) == 1) and (digitalRead(CHARGING_GPIO) == 1)) charging_int = 3; // OFF ??

          LOG2("[%s]: charging=%s  charging_int=%d  CHARGING_GPIO=%d  POWER_GPIO=%d\n",__func__,charging_states[charging_int],charging_int,charging_gpio_state,power_gpio_state);
          #ifdef DEBUG
            Serial.printf("[%s]: charging=%s  charging_int=%d  CHARGING_GPIO=%d  POWER_GPIO=%d\n",__func__,charging_states[charging_int],charging_int,charging_gpio_state,power_gpio_state);
          #endif
        #endif
        // check charging END

        vTaskDelay(pdMS_TO_TICKS(1000));

        #ifdef DEBUG_XTASKS
          uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
          // Serial.print("check_volts uxHighWaterMark=");Serial.println(uxHighWaterMark);
          LOG1("[%s]: check_volts uxHighWaterMark=%d\n",__func__,uxHighWaterMark);
        #endif
      }
}


// update firmware
// FW upgrade wrapper
void do_update()
{
  Serial.printf("[%s]: FW UPDATE starting...\n",__func__);
  int update_firmware_status = -1;
  update_firmware_status=update_firmware_prepare();
  if (update_firmware_status == 0)
  {
    Serial.printf("[%s]: RESTARTING - FW update SUCCESSFULL\n\n",__func__);
    // blink slowly when FW upgrade successfull
    for (int i=0;i<3;i++)
    {
      #ifdef ERROR_RED_LED_GPIO
        digitalWrite(ERROR_RED_LED_GPIO,LOW);
        delay(100);
        digitalWrite(ERROR_RED_LED_GPIO,HIGH);
        delay(30);
      #elif defined(STATUS_LED_GPIO)
        digitalWrite(STATUS_LED_GPIO,LOW);
        delay(100);
        digitalWrite(STATUS_LED_GPIO,HIGH);
        delay(30);
        // digitalWrite(ERROR_RED_LED_GPIO,LOW);
      #endif
    }

  } else
  {
    Serial.printf("[%s]: FW update failed - reason: %d\n",__func__,update_firmware_status);
    #ifdef ERROR_RED_LED_GPIO
      // sos(ERROR_RED_LED_GPIO);
    #elif defined(STATUS_LED_GPIO)
      // sos(STATUS_LED_GPIO);
    #endif
  }
  // 
  do_restart_esp("FW update finished");
}

// real update
void updateFirmware(uint8_t *data, size_t len)
{
  // blink ERROR_RED_LED_GPIO or...
  #ifdef ERROR_RED_LED_GPIO
    if (blink_led_status) {
      blink_led_status=LOW;
      digitalWrite(ERROR_RED_LED_GPIO,blink_led_status);
    } else {
      blink_led_status=HIGH;
      digitalWrite(ERROR_RED_LED_GPIO,blink_led_status);
    }
  #else
    // ...blink STATUS_LED_GPIO
    #ifdef STATUS_LED_GPIO
      if (blink_led_status) {
        blink_led_status=LOW;
        digitalWrite(STATUS_LED_GPIO,blink_led_status);
      } else {
        blink_led_status=HIGH;
        digitalWrite(STATUS_LED_GPIO,blink_led_status);
      }
    #endif
  #endif

  Update.write(data, len);
  fw_currentLength += len;
  old_update_progress=update_progress;
  update_progress=(fw_currentLength * 100) / fw_totalLength;
  if (update_progress>old_update_progress){
    if (update_progress % 5 == 0){ //it prints every 5%
      Serial.printf("[%s]: FW update: %d%%\n",__func__,update_progress);
    }
  }
  // if current length of written firmware is not equal to total firmware size, repeat
  if(fw_currentLength != fw_totalLength) return;
  Update.end(true);
  Serial.printf("\n[%s]: Update Success, Total Size: %d bytes\n",__func__,fw_currentLength);
}

// download from webserver
int update_firmware_prepare()
{
  char mac_new_char_short[13]; // to find the path for FW file
  // get folder with bin file from mac address that is in devices_confi.gh in variableL  uint8_t FixedMACAddress[] = {0x1A, 0xFF, 0x01, 0x01, 0x01, 0x01};
  snprintf(mac_new_char_short, sizeof(mac_new_char_short), "%02x%02x%02x%02x%02x%02x",FixedMACAddress[0], FixedMACAddress[1], FixedMACAddress[2], FixedMACAddress[3], FixedMACAddress[4], FixedMACAddress[5]);

  char firmware_file[255];
  snprintf(firmware_file,sizeof(firmware_file),"%s/%s/%s",UPDATE_FIRMWARE_HOST,mac_new_char_short,FW_BIN_FILE);

  fw_totalLength=0;
  fw_currentLength=0;
  Serial.printf("[%s]: uploading file: %s\n",__func__,firmware_file);
  firmware_update_client.begin(firmware_file);
  int resp = firmware_update_client.GET();
  Serial.printf("[%s]: Response: %d\n",__func__,resp);
  // If file is reachable, start downloading
  if(resp == 200){
    // get length of document (is -1 when Server sends no Content-Length header)
    fw_totalLength = firmware_update_client.getSize();
    // transfer to local variable
    int len = fw_totalLength;
    // this is required to start firmware update process
    Update.begin(UPDATE_SIZE_UNKNOWN);
    Serial.printf("[%s]: FW Size: %lu bytes\n",__func__,fw_totalLength);

    // create buffer for read
    uint8_t buff[128] = { 0 };
    // get tcp stream
    WiFiClient * stream = firmware_update_client.getStreamPtr();
    // read all data from server
    Serial.printf("[%s]: Updating firmware progress:\n",__func__);
    while(firmware_update_client.connected() && (len > 0 || len == -1))
    {
      // get available data size
      size_t size = stream->available();
      if(size) {
        // read up to 128 byte
        int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
        // pass to function
        updateFirmware(buff, c);
        if(len > 0) {
           len -= c;
        }
      }
      delay(1);
      }
  }else
  {
    Serial.printf("[%s]: Cannot download firmware file. Only HTTP response 200: OK is supported. Double check firmware location.\n",__func__);
    Serial.printf("[%s]: firmware update prepare UNSUCESSFUL\n",__func__);
    return resp;
  }
  firmware_update_client.end();
  Serial.printf("[%s]: firmware update prepare SUCESSFULL\n",__func__);
  return 0;
}
// update firmware END

void change_mac()
{
  WiFi.mode(WIFI_STA);
  WiFi.macAddress(mac_org);

  snprintf(mac_org_char, sizeof(mac_org_char), "%02x:%02x:%02x:%02x:%02x:%02x",mac_org[0], mac_org[1], mac_org[2], mac_org[3], mac_org[4], mac_org[5]);
  Serial.printf("[%s]: OLD MAC: %s\n",__func__,mac_org_char);

  Serial.printf("[%s]: changing MAC...",__func__);
  if (esp_wifi_set_mac(WIFI_IF_STA, &FixedMACAddress[0]) == ESP_OK) Serial.println("SUCCESSFULL"); else Serial.println("FAILED");

  WiFi.macAddress(mac_new);
  snprintf(mac_new_char, sizeof(mac_new_char), "%02x:%02x:%02x:%02x:%02x:%02x",mac_new[0], mac_new[1], mac_new[2], mac_new[3], mac_new[4], mac_new[5]);
  Serial.printf("[%s]: NEW MAC: %s\n",__func__,mac_new_char);
}

void make_serial_number(char *org_mac, char*new_mac, char *buff)
{
  String mac_org_str = String(org_mac);
  String mac_new_str = String(new_mac);

  mac_org_str.replace(":","");
  mac_new_str.replace(":",""); 
  mac_new_str.remove(0,6);

  // find the size of the new string formatted
  size_t nbytes = snprintf(NULL,0,"%s_%s",mac_org_str,mac_new_str) +1;

  // final serial_number in format: "oldmac_6charofnewmac"
  snprintf(buff,nbytes,"%s_%s",mac_org_str,mac_new_str);

  #ifdef DEBUG
    Serial.printf("[%s]: serial_number: %s\n",__func__,buff);
  #endif
}

void make_fw_version(const char *fw_numeric, char *fw_with_date_time)
{
  int month, day, year, zh_hour,zh_minute, short_y;
  static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  // day, year done
  sscanf(__DATE__, "%s %d %d", fw_with_date_time, &day, &year);
  // short year
  short_y=(((year / 10U) % 10)* 10)  + ((year / 1U) % 10);
  //month done
  month = (strstr(month_names, fw_with_date_time)-month_names)/3+1;
  // time
  String time_str = String(__TIME__);
  time_str.replace(":"," ");
  char time_chr[30];
  snprintf(time_chr,sizeof(time_chr),"%s",time_str);
  sscanf(time_chr, "%d %d", &zh_hour, &zh_minute);

  // tmp char
  char fw_char[20];
  // numeric to string
  String fw_str = String(fw_numeric);
  // dots to space
  fw_str.replace("."," ");
  // string to char
  snprintf(fw_char,sizeof(fw_char),"%s",fw_str);
  int major,minor,patch;
  // digits to int
  sscanf(fw_char, "%d %d %d", &major, &minor, &patch);
  snprintf(fw_char,sizeof(fw_char),"%d",major*100+minor*10+patch);

  // final fw_version in format: FW.DATE.TIME (of compilation) where FW= major(hundreds)+minor(tens)+patch(units)
  size_t nbytes =   snprintf(NULL,0,"%s.%02d%02d%02d.%02d%02d",fw_char, short_y, month, day,zh_hour,zh_minute)+1;
  snprintf(fw_with_date_time,nbytes,"%s.%02d%02d%02d.%02d%02d",fw_char, short_y, month, day,zh_hour,zh_minute);

  #ifdef DEBUG
    Serial.printf("[%s]: fw_version: %s\n",__func__,fw_with_date_time);
  #endif
}

// REMOTE DEVICES 
struct UpdateData : Service::AccessoryInformation
{
  SpanCharacteristic *md_version;

  SpanPoint *remoteData;
  const char *name;
  const char *macAddress;
  
  UpdateData(const char *name, const char*macAddress) : Service::AccessoryInformation()
  {
    this->name=name;
    this->macAddress=macAddress;
    // not updatable
    new Characteristic::Identify();
    new Characteristic::Name(name);
    new Characteristic::SerialNumber(macAddress);  
    
    // updatable
    md_version    = new Characteristic::FirmwareRevision("");
    remoteData    = new SpanPoint(macAddress,0,sizeof(myData));    // create a SpanPoint with send size=0 and receive size=sizeof(myData)
  }

  void loop()
  {                                       // if there is data from the remote sensor
    if(remoteData->get(&myData))
    {                                       // if there is data from the remote sensor
      // should RED LED blink when new message comes? maybe not..
      #ifdef BLINK_ERROR_LED_ON_RECEIVED_DATA
        #ifdef ERROR_RED_LED_GPIO
          digitalWrite(ERROR_RED_LED_GPIO,HIGH);
        #endif
      #endif
      // but maybe the Status LED?
      #ifdef BLINK_STATUS_LED_ON_RECEIVED_DATA
        if (digitalRead(homeSpan.getStatusPin())) digitalWrite(homeSpan.getStatusPin(),LOW);
      #endif

      LOG0("\n[%s]: Update from device=%s...\n",__func__,macAddress);
      temperature_value     = myData.md_temp;       // update temperature
      temperature_received  = true;
      humidity_value        = myData.md_hum;        // update humidity
      humidity_received     = true;
      light_value           = myData.md_light;      // update light
      light_received        = true;
      md_mcu_model_value    = myData.md_mcu_model;  // update model
      bat_value_pct         = myData.md_bat;        // update battery percent
      bat_received          = true;
      md_charging_value     = myData.md_charging;   // update charging
      charging_received     = true;

      snprintf(md_version_value,sizeof(md_version_value),"%s",myData.md_version); // update version
      u_int32_t free_m = ESP.getFreeHeap();
      LOG0  ("[%s]:  Raw data from device %s just arrived: temp=%0.2fC, hum=%0.2f%%, light=%0.2flx, bat_value_pct=%d%%, charging=%d, model=%d (%s), version=%s\n",__func__,name,temperature_value,humidity_value,light_value,bat_value_pct,md_charging_value,md_mcu_model_value,models[md_mcu_model_value],md_version_value);
      WEBLOG("Free=%dbytes %s: T=%0.2f H=%0.2f L=%0.2f Bat=%d%% CH=%d Board=%d v=%s\n",free_m,name,temperature_value,humidity_value,light_value,bat_value_pct,md_charging_value,md_mcu_model_value,md_version_value);


      md_version->setString(md_version_value);                    // update version

      device_timeout = false;      // clean timeout
      
    } else 
    // DON'T PRINT HERE ANYTHING!
    if (remoteData->time()>(UPDATE_TIMEOUT_S * 1000)) 
    { 
      device_timeout = true;
    }
  }
};

struct RemoteTempSensor : Service::TemperatureSensor
{
  SpanCharacteristic *temp;
  SpanCharacteristic *fault;
  
  const char *name;
  const char *macAddress;

  RemoteTempSensor(const char *name, const char*macAddress) : Service::TemperatureSensor()
  {
    this->name=name;
    this->macAddress=macAddress;
    temp=new Characteristic::CurrentTemperature(0);           // set initial temperature
    temp->setRange(-50,150);                                  // expand temperature range to allow negative values
    fault=new Characteristic::StatusFault(0);                 // set initial state = no fault
  }

  void loop()
  {
    if (temperature_received)
    {
      if (temperature_value < -90)
      {
        fault->setVal(1);                                     // set fault state = FAULT
        LOG0("[%s]: ERROR: temperature from sensor %s on device %s INVALID=%f\n",__func__,name,macAddress,temperature_value);
        temperature_received = false;
      } else
      {
        fault->setVal(0);                                     // set fault state = OK                      
        temp->setVal(temperature_value);              
        LOG1("[%s]: Temperature from sensor %s on device %s=%f, fault=%d\n",__func__,name,macAddress,temperature_value,fault->getVal());
        temperature_received = false;
      }
    } else 
    // DON'T PRINT HERE ANYTHING!
    if (device_timeout) 
    {
      fault->setVal(1);   
    }
  }
};

struct RemoteHumSensor : Service::HumiditySensor
{
  SpanCharacteristic *hum;
  SpanCharacteristic *fault;
  const char *name;
  const char *macAddress;

  RemoteHumSensor(const char *name, const char*macAddress) : Service::HumiditySensor()
  {
    this->name=name;
    this->macAddress=macAddress;
    hum=new Characteristic::CurrentRelativeHumidity(0);     // set initial humidity
    hum->setRange(0,100);                                   // expand hum range
    fault=new Characteristic::StatusFault(0);               // set initial state = no fault
  }

  void loop()
  {
    if (humidity_received)
    {
      if (humidity_value < 0)
      {
        fault->setVal(1);                                     // set fault state = FAULT
        LOG0("[%s]: ERROR: Humidity from sensor %s on device %s INVALID=%f\n",__func__,name,macAddress,humidity_value);
        humidity_received = false;
      } else
      {
        fault->setVal(0);                                     // set fault state = OK
        hum->setVal(humidity_value);              // update humidity
        LOG1("[%s]: Humidity from sensor %s on device%s=%f\n",__func__,name,macAddress,humidity_value);
        humidity_received = false;
      }
    } else
    // DON'T PRINT HERE ANYTHING!
    if (device_timeout) 
    {
      fault->setVal(1);   
    }
  }
};

struct RemoteLightSensor : Service::LightSensor
{
  SpanCharacteristic *lux;
  SpanCharacteristic *fault;
  const char *name;
  const char *macAddress;

  RemoteLightSensor(const char *name, const char*macAddress) : Service::LightSensor()
  {
    this->name=name;
    this->macAddress=macAddress;
    lux=new Characteristic::CurrentAmbientLightLevel(0.1);    // set initial light
    fault=new Characteristic::StatusFault(0);                 // set initial state = fault
  }

  void loop()
  {
    if (light_received)
    {
      if (light_value < 0)
      {
        fault->setVal(1);                                     // set fault state = FAULT
        LOG0("[%s]: ERROR: Light from sensor %s on device %s INVALID=%f\n",__func__,name,macAddress,light_value);
        light_received = false;
      } else
      {
        fault->setVal(0);                                     // set fault state = OK
        lux->setVal(light_value);                             // update light
        LOG1("[%s]: Light from sensor %s on device %s=%f\n",__func__,name,macAddress,light_value);
        light_received = false;
      }
    } else 
    // DON'T PRINT HERE ANYTHING!
    if (device_timeout) 
    {
      fault->setVal(1);   
    }
  }

};

struct RemoteBattery : Service::BatteryService
{
  SpanCharacteristic *battery_level;
  SpanCharacteristic *charging_state;
  SpanCharacteristic *low_battery;
  const char *name;
  const char *macAddress;

  RemoteBattery(const char *name, const char*macAddress) : Service::BatteryService()
  {
    this->name=name;
    this->macAddress=macAddress;
    new Characteristic::Name("Battery sensor");
    battery_level     = new Characteristic::BatteryLevel(0);                     // set initial state to 0%
    charging_state    = new Characteristic::ChargingState(0);                    // set initial state to "not charging" 
    low_battery       = new Characteristic::StatusLowBattery(1);                 // set initial state to true
  }

  void loop()
  {
    if (bat_received)
    {
      battery_level->setVal(bat_value_pct);   
      if (bat_value_pct < 30) 
      {
        low_bat=1;
      } else
      {
        low_bat=0;
      }
      low_battery->setVal(low_bat);  
      bat_received = false;

      if (charging_received)
      {
        if (md_charging_value > 0)
          charging_state->setVal(1);
        else 
          charging_state->setVal(0);
      }
      LOG1("[%s]: Battery data from sensor %s on device %s: battery level=%d%%, low_battery=%d, charging=%d\n",__func__,name,macAddress,bat_value_pct,low_bat,md_charging_value);
      
      charging_received = false;

      // should RED LED blink when new message comes? maybe not..
      #ifdef BLINK_ERROR_LED_ON_RECEIVED_DATA
        #ifdef ERROR_RED_LED_GPIO
          digitalWrite(ERROR_RED_LED_GPIO,LOW);
        #endif
      #endif
      // but maybe the Status LED?
      #ifdef BLINK_STATUS_LED_ON_RECEIVED_DATA
        digitalWrite(homeSpan.getStatusPin(),HIGH);
      #endif
    } else 
    // DON'T PRINT HERE ANYTHING!
    if (device_timeout) 
    {
      device_timeout = false; // clear timeout as all data already done for this device and this timeout is not relevant to next devices
    }
  }
};
// REMOTE DEVICES END


// BRIDGE
struct UpdateBattery : Service::BatteryService
{
  SpanCharacteristic *battery_level;
  SpanCharacteristic *charging_state;
  SpanCharacteristic *low_battery;

  UpdateBattery() : Service::BatteryService()
  {
    new Characteristic::Name("Battery sensor");
    battery_level     = new Characteristic::BatteryLevel(50);                    // set initial state to 50%
    charging_state    = new Characteristic::ChargingState(0);                    // set initial state to "not charging"
    low_battery       = new Characteristic::StatusLowBattery(0);                 // set initial state to false (no alarm)
  }

  void loop()
  {
    
    if (battery_level->timeVal()>(BATTERY_INTERVAL_S * 1000))
    {
      #if defined(CHARGING_GPIO) and defined(POWER_GPIO)
        LOG1("[%s]: Bridge charging status=%d(%s)\n",__func__,charging_int,charging_states[charging_int]);
        if ((charging_int == 1) or (charging_int == 2))
        {
          charging_state->setVal(1);
        }
        else 
        {
          charging_state->setVal(0);
        }
      // #else // don't enable - it floods the screen
      //   LOG0("[%s]: Checking charging DISABLED!\n",__func__);
      #endif 

      #if (USE_MAX17048 == 1)
        if (check_battery)
        {
          battery_level->setVal(bat_pct);  
          LOG1("[%s]: Bridge battery status: volts=%0.2fV, percent=%0.2f%%\n",__func__,volts,bat_pct);
          if (bat_pct < LOW_BATTERY_THRESHOLD)
          {
            low_battery->setVal(1);  
            LOG0("[%s]: Bridge battery CRITICAL status: volts=%0.2fV, percent=%0.2f%%, charging=%d(%s)\n",__func__,volts,bat_pct,charging_int,charging_states[charging_int]);
          } else 
          {
            low_battery->setVal(0);  
          }
        } else 
        {
          low_battery->setVal(0);  
        }
      #endif
    }
  }
};

struct DEV_Identify : Service::AccessoryInformation 
{
  int nBlinks;                    // number of times to blink built-in LED in identify routine
  SpanCharacteristic *identify;   // reference to the Identify Characteristic
  
  DEV_Identify(const char *name, const char *manu, const char *sn, const char *model, const char *version, int nBlinks) : Service::AccessoryInformation()
  {
    
    new Characteristic::Name(name);                   // create all the required Characteristics with values set based on above arguments
    new Characteristic::Manufacturer(manu);
    new Characteristic::SerialNumber(sn);    
    new Characteristic::Model(model);
    new Characteristic::FirmwareRevision(version);
    identify=new Characteristic::Identify();          // store a reference to the Identify Characteristic for use below

    this->nBlinks=nBlinks;                            // store the number of times to blink the LED
  }

  boolean update()
  { 
    #ifdef ERROR_RED_LED_GPIO    
        LOG0("[%s]: bridge called to identify itself, blinking RED LED...\n",__func__);
        for(int i=0;i<nBlinks;i++)
        {
            digitalWrite(ERROR_RED_LED_GPIO,HIGH);
            delay(50);
            digitalWrite(ERROR_RED_LED_GPIO,LOW);
            delay(50);
        }
    #elif defined(STATUS_LED_GPIO)
        LOG0("[%s]: bridge called to identify, blinking status LED...\n",__func__);
        for(int i=0;i<nBlinks;i++)
        {
            digitalWrite(STATUS_LED_GPIO,HIGH);
            delay(50);
            digitalWrite(STATUS_LED_GPIO,LOW);
            delay(50);
        }
    #else 
        LOG1("[%s]: no LED defined - failed\n",__func__);
        return false;
    #endif
    LOG0("[%s]: bridge identified\n",__func__);


    unsigned long free_mem = ESP.getFreeHeap();
    LOG0("[%s]: free heap=%u bytes\n",__func__,free_mem);
    if (free_mem > 66000) 
    {
      do_update();
    } else
    {
      LOG0("[%s]: NOT ENOUGH MEMORY FOR FW UPDATE!!!\n",__func__);
      return false;
    }
    return true;
  }

};
// BRIDGE END

void setup() 
{
  Serial.begin(115200);
  delay(50);
  Serial.printf("\n======= S T A R T =======\n");

  // apply new MAC address
  change_mac(); // it provides: mac_org_char,  mac_new_char changed globally

  // make new SN char based on old and new MACs
  char serial_number[20];
  make_serial_number(mac_org_char,mac_new_char,serial_number);

  // make fw_version char based on BRIDGE_FW and date/time of compilation 
  char fw_version[20];
  make_fw_version(BRIDGE_FW, fw_version);

  // intro
  LOG0("[%s]: HOSTNAME: %s\n",__func__,HOSTNAME);
  LOG0("[%s]: FW VERSION: %s\n",__func__,BRIDGE_FW);
  LOG0("[%s]: COMPILATION TIME: %s %s\n",__func__,__DATE__,__TIME__);
  LOG0("[%s]: BOARD TYPE: %s\n",__func__,MODEL);
  LOG0("[%s]: Free heap=%u bytes\n",__func__,ESP.getFreeHeap());

  LOG0("[%s]: BRIDGE_FW on HomeKit: %s\n",__func__,fw_version);
  LOG0("[%s]: SERIAL NUMBER: %s\n",__func__,serial_number);

  #ifdef ERROR_RED_LED_GPIO
    pinMode(ERROR_RED_LED_GPIO,OUTPUT);
  #endif

  homeSpan.setApSSID(HOSTNAME);
  homeSpan.setApPassword("");   // no password for CP
  homeSpan.enableOTA();         // homespan-ota
  homeSpan.enableAutoStartAP(); // AP on startup if no WiFi credentials
  homeSpan.setHostNameSuffix("");;
  // homeSpan.setStatusAutoOff(5);  // turn OFF LED - not good as no info about the life of bridge then 

  // start checking charging
  #if defined(CHARGING_GPIO) and defined(POWER_GPIO)
    #ifdef DEBUG
      Serial.printf("[%s]: CHARGING_GPIO and POWER_GPIO enabled\n",__func__);
    #endif
    pinMode(CHARGING_GPIO, INPUT_PULLUP);   // must be PULLUP as TP4056 is Active LOW
    pinMode(POWER_GPIO, INPUT_PULLUP);      // must be PULLUP as TP4056 is Active LOW
  #else
    #ifdef DEBUG
      Serial.printf("[%s]: checking CHARGING DISABLED\n",__func__);
    #endif
  #endif
  // start start checking charging END  

  #if defined(CUSTOM_SDA_GPIO) and defined(CUSTOM_SCL_GPIO)
    #ifdef DEBUG
        Serial.printf("[%s]: CUSTOM I2C GPIO pins applied: SDA=%d, SCL=%d\n",__func__,CUSTOM_SDA_GPIO,CUSTOM_SCL_GPIO);
    #endif
      Wire.setPins(CUSTOM_SDA_GPIO,CUSTOM_SCL_GPIO);
  #endif
  Wire.begin();
  delay(100); 

  // MAX17048 - fuel gauge
  #if (USE_MAX17048 == 1)
    unsigned long max17048_begin_time = millis();
    #ifdef DEBUG
      lipo.enableDebugging();
      Serial.printf("[%s]: start USE_MAX17048\n",__func__);
    #endif

    if (lipo.begin())
    {
      // lipo.quickStart();     // not needed rather, MAX17048 can recalculate in the time
      check_battery = true;
      lipo.disableHibernate();
      #ifdef DEBUG
        Serial.printf("[%s]: start MAX17048 OK\n",__func__);
      #endif
      xReturned_check_volts = xTaskCreate(check_volts, "check_volts", 2500, NULL, 1, &check_volts_handle);
      if( xReturned_check_volts != pdPASS )
      {
        Serial.printf("[%s]: check_volts task not created !!!ś\n",__func__);
      } else
      {
        Serial.printf("[%s]: check_volts task created\n",__func__);
      } 
    } else 
    {
      Serial.printf("[%s]: MAX17048 NOT detected ... Check your wiring or I2C ADDR!\n",__func__);
      check_battery = false;
    } 

  #else
    #ifdef DEBUG
      Serial.printf("[%s]: DONT USE_MAX17048\n",__func__);
    #endif
  #endif  
  // MAX17048 - fuel gauge END

  homeSpan.setLogLevel(LOG_LEVEL);
  homeSpan.setControlPin(CONTROL_PIN);
  homeSpan.setStatusPin(STATUS_LED_GPIO);
  homeSpan.enableOTA();
  homeSpan.enableWebLog(MAX_WEBLOG_ENTRIES,NTP_SERVER,"UTC","stats");   // opens webpage with logs
  homeSpan.setTimeServerTimeout(NTP_SERVER_TIMEOUT_S);
  homeSpan.setSketchVersion(fw_version);

// BRIDGE:
  new SpanAccessory();
    new DEV_Identify(HOSTNAME,MANUFACTURER,serial_number,MODEL,fw_version,IDENTIFY_BLINKS);
    new UpdateBattery();

// add accessories only for DEVICE_ID = 1 as of now - don't mess while testing other boards
  #if (DEVICE_ID == 1)      
  // DEVICE 1
    new SpanAccessory();
      new UpdateData(DEVICE_ID_1_NAME,DEVICE_ID_1);                 
      new RemoteTempSensor("Temperature Sensor",DEVICE_ID_1);        
      new RemoteHumSensor("Humidity Sensor",DEVICE_ID_1);        
      new RemoteLightSensor("Light Sensor",DEVICE_ID_1);  
      new RemoteBattery(DEVICE_ID_1_NAME,DEVICE_ID_1);    

  // DEVICE 2
    new SpanAccessory();
      new UpdateData(DEVICE_ID_2_NAME,DEVICE_ID_2);                 
      new RemoteTempSensor("Temperature Sensor",DEVICE_ID_2);        
      new RemoteHumSensor("Humidity Sensor",DEVICE_ID_2);        
      new RemoteLightSensor("Light Sensor",DEVICE_ID_2);       
      new RemoteBattery(DEVICE_ID_2_NAME,DEVICE_ID_2);     

  // DEVICE 3
    new SpanAccessory();
      new UpdateData(DEVICE_ID_3_NAME,DEVICE_ID_3);                 
      new RemoteTempSensor("Temperature Sensor",DEVICE_ID_3);        
      new RemoteHumSensor("Humidity Sensor",DEVICE_ID_3);        
      new RemoteLightSensor("Light Sensor",DEVICE_ID_3);  
      new RemoteBattery(DEVICE_ID_3_NAME,DEVICE_ID_3);    


  // // DEVICE 4
  //   new SpanAccessory();
  //     new UpdateData(DEVICE_ID_4_NAME,DEVICE_ID_4);                 
  //     new RemoteTempSensor("Temperature Sensor",DEVICE_ID_4);        
  //     new RemoteHumSensor("Humidity Sensor",DEVICE_ID_4);        
  //     new RemoteLightSensor("Light Sensor",DEVICE_ID_4);  
  //     new RemoteBattery(DEVICE_ID_4_NAME,DEVICE_ID_4);    

  #endif

  // magic starts here
  homeSpan.begin(Category::Bridges,HOSTNAME,HOSTNAME);

  //OTA in Setup - must be after homeSpan.begin as wifi starts there
  #ifdef OTA_ACTIVE
    strlcpy(ota_user, OTA_USER, sizeof(ota_user));
    strlcpy(ota_password, OTA_PASSWORD, sizeof(ota_password));
    Serial.printf("[%s]: Enabling OTA:...\n",__func__);
    Serial.printf("[%s]: user=%s, password=%s, URL=http://%s.local:8080/update\n",__func__,ota_user,ota_password,HOSTNAME);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      String introtxt = "This is device "+String(DEVICE_ID) +", Version: " + String(BRIDGE_FW);
      request->send(200, "text/plain", String(introtxt));
    });
    AsyncElegantOTA.begin(&server, ota_user,ota_password);    // Start ElegantOTA
    server.begin();
  #else
    Serial.printf("[%s]: !!! OTA NOT ENABLED !!! \n",__func__);
  #endif
  //OTA in Setup END

  Serial.printf("[%s]: ========== Setup finished ==========\n\n",__func__);
}

void loop()
{
  homeSpan.poll();
}