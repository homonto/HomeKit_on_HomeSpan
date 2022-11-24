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




/*
  todo:
  - all configuration in Preferences (or JSON?)
  - add ORG and FAKE MAC to Captive Portal
*/

#define FW_VERSION          "0.2.0"
#define CLIENT              "001-fv"


// #define DEVICE_ID           1 // C3 - first built -                    "homekit-sensor-1"
#define DEVICE_ID           2 // S2 - without the box - development -  "homekit-sensor-2"
// #define DEVICE_ID           3 // C3 - second built -                   "homekit-sensor-3"


// #define DEBUG

// ****************  EVERYTHONG BELOW ALL IS COMMON FOR ANY ESP32 *********************
// #define WIFI_CHANNEL                8   // in my house
#define WAIT_FOR_WIFI               10  // in seconds, for upgrade firmware
#define uS_TO_S_FACTOR              1000000ULL
#define COMPILATION_TIME            (__DATE__ ", " __TIME__)
#define BRIDGE                      "1A:FF:01:01:01:01" // bridge fake MAC

#define CP_TIMEOUT_S                180  // CP and AP will terminate after this time
#define SLEEP_TIME_H_BATTERY_EMPTY  24  // sleep hours when battery empty
#define MAX17048_DELAY_ON_RESET_MS  200 // as per datasheet: needed before next reading from MAX17048 after reset, only in use when reset/battery change

#include <Arduino.h>
#include "devices_config.h"
#include "passwords.h"

// MAX17048 - battery fuel gauge, I2C
#if (USE_MAX17048 == 1)
  #include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
  SFE_MAX1704X lipo(MAX1704X_MAX17048);
#endif

// sht31 - temperature and humidity, I2C
#if (USE_SHT31 == 1)
  #include "Adafruit_SHT31.h"
  Adafruit_SHT31 sht31 = Adafruit_SHT31();
#endif

// lux from TSL2561 - light sensor, I2C
#if (USE_TSL2561 == 1)
  #include <SparkFunTSL2561.h>
  SFE_TSL2561 light;
#endif

// other libraries and variables
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>

#include "HomeSpan.h"
SpanPoint *mainDevice;

typedef struct struct_message          // 24 bytes
{
  float md_temp;
  float md_hum;
  float md_light;
  uint8_t md_bat;
  char md_version[10];
  uint8_t md_mcu_model;
} struct_message;

struct_message myData;

// Firmware update
char fake_mac[18];
#include <HTTPClient.h>
#include <Update.h>
#if   (BOARD_TYPE == 1)
  #define FW_BIN_FILE "env_sensors.esp32.bin"
#elif (BOARD_TYPE == 2)
  #define FW_BIN_FILE "env_sensors.esp32s2.bin"
#elif (BOARD_TYPE == 3)
  #define FW_BIN_FILE "env_sensors.esp32s3.bin"
#elif (BOARD_TYPE == 4)
  #define FW_BIN_FILE "env_sensors.esp32c3.bin"
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

// auxuliary variables:
// sleep:
unsigned long start_time, end_time, work_time;
// tasks
TaskHandle_t led_blink_handle = NULL;
BaseType_t xReturned_led_blink;
// timers
#include "freertos/timers.h"
TimerHandle_t cp_timer_handle = NULL;
// int interval = 5000; //ms
int id = 1;

// check reasons to wake up and start
esp_sleep_wakeup_cause_t wakeup_reason;
byte boot_reason; // esp_reset_reason_t ???
uint64_t wakeup_gpio_mask;
byte wakeup_gpio;

// declarations
void change_mac();
// void setup_wifi();
void sos(int led);
void do_update();
void updateFirmware(uint8_t *data, size_t len);
int update_firmware_prepare();
void hibernate(bool force, int final_sleeping_time_s);
void do_esp_restart();
void led_blink(void *pvParams);
void cp_timer( TimerHandle_t xTimer );

// CAPTIVE PORTAL
#include <nvs_flash.h>
#include <DNSServer.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include <Preferences.h>

void connect_wifi();
void setupServer();
void WiFiSoftAPSetup();
void WiFiStationSetup(String rec_ssid, String rec_password, String rec_channel, String rec_sleep_s);
void StartCaptivePortal();
void reset_wifi_credentials();
void display_wifi_credentials();

Preferences preferences;
DNSServer dnsServer;
AsyncWebServer server(80);

String ssid, old_ssid;
String password,old_password;
String channel,old_channel;
String sleeptime_s_str,old_sleeptime_s_str;
bool is_setup_done = false;
bool valid_ssid_received = false;
bool valid_channel_received = false;
bool valid_password_received = false;
bool valid_sleeptime_s_str_received = false;
bool wifi_timeout = false;
int sleeptime_s = SLEEP_TIME_S;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Captive Portal</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h3>Captive Portal</h3>
  <h1>Leave blank what you don't want to change - it will use the stored values</h1>
  <br><br>
  <form action="/get">
    <br>
    SSID: <input type="text" name="ssid">
    <br>
    Password: <input type="password" name="password">
    <br>
    Channel (leave empty if Access Point/Router is set to Auto-Channel): <input type="text" name="channel">
    <br>
    Sleep time in seconds [1-3600] (leave empty = DEFAULT(180s)): <input type="text" name="sleeptime_s_str">

    <input type="submit" value="Submit">
  </form>
</body></html>)rawliteral";

class CaptiveRequestHandler : public AsyncWebHandler {
  public:
    CaptiveRequestHandler() {}
    virtual ~CaptiveRequestHandler() {}

    bool canHandle(AsyncWebServerRequest *request) {
      //request->addInterestingHeader("ANY");
      return true;
    }

    void handleRequest(AsyncWebServerRequest *request) {
      request->send_P(200, "text/html", index_html);
    }
};

void reset_wifi_credentials()
{
  Serial.printf("[%s]: Storring old configuration data...\n",__func__);
  preferences.begin("wifi-secrets", false);

  old_ssid = preferences.getString("rec_ssid", "Sample_SSID");
  old_channel = preferences.getString("rec_channel", "12");
  old_password = preferences.getString("rec_password", "abcdefgh");
  old_sleeptime_s_str = preferences.getString("rec_sleep_s", "5");

  Serial.printf("[%s]: old_ssid:            %s\n",__func__,old_ssid);
  Serial.printf("[%s]: old_channel:         %s\n",__func__,old_channel);
  Serial.printf("[%s]: old_password:        ",__func__); Serial.println(old_password);
  Serial.printf("[%s]: old_sleeptime_s_str: %s seconds\n",__func__,old_sleeptime_s_str);

  Serial.printf("[%s]: Resetting wifi credentials...\n",__func__);


  preferences.clear();
  preferences.end();
  delay(1);
}

void display_wifi_credentials()
{
  Serial.printf("[%s]: Saved configuration data...\n",__func__);
  preferences.begin("wifi-secrets", true);    // read only

  old_ssid = preferences.getString("rec_ssid", "Sample_SSID");
  old_channel = preferences.getString("rec_channel", "12");
  old_password = preferences.getString("rec_password", "abcdefgh");
  old_sleeptime_s_str = preferences.getString("rec_sleep_s", "5");

  Serial.printf("[%s]: ssid:            %s\n",__func__,old_ssid);
  Serial.printf("[%s]: channel:         %s\n",__func__,old_channel);
  Serial.printf("[%s]: password:        ",__func__); Serial.println(old_password);
  Serial.printf("[%s]: sleeptime_s_str: %s\n",__func__,old_sleeptime_s_str);

  preferences.end();
}

void setupServer()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send_P(200, "text/html", index_html);
    Serial.println("Client Connected");
  });
// Serial.printf("[%s]: Client Connected\n",__func__);
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest * request)
  {
    String inputMessage;
    String inputParam;

    if (request->hasParam("ssid"))
    {
      inputMessage = request->getParam("ssid")->value();
      inputParam = "ssid";
      ssid = inputMessage;
      Serial.printf("[%s]: SSID received: %s\n",__func__,ssid);
      if (strlen(ssid.c_str()) == 0)
      {
        ssid = old_ssid;
        Serial.printf("[%s]: SSID is blank, reusing the old value: %s\n",__func__,ssid);
      } else
      {
        ssid = inputMessage;
      }
      valid_ssid_received = true;
    }

    if (request->hasParam("password"))
    {
      inputMessage = request->getParam("password")->value();
      inputParam = "password";
      password = inputMessage;
      Serial.printf("[%s]: password received: ",__func__);Serial.println(password);
      if (strlen(password.c_str()) == 0)
      {
        password = old_password;
        Serial.printf("[%s]: password is blank, reusing the old value: ",__func__); Serial.println(password);
      } else
      {
        password = inputMessage;
      }
      valid_password_received = true;
    }

    if (request->hasParam("channel"))
    {
      inputMessage = request->getParam("channel")->value();
      inputParam = "channel";
      channel = inputMessage;
      Serial.printf("[%s]: channel received: %s\n",__func__,channel);
      if (strlen(channel.c_str()) == 0)
      {
        channel = old_channel;
        Serial.printf("[%s]: channel is blank, reusing the old value: %s\n",__func__,channel);
      } else
      {
        channel = inputMessage;
      }
      valid_channel_received = true;
    }

    if (request->hasParam("sleeptime_s_str"))
    {
      inputMessage = request->getParam("sleeptime_s_str")->value();
      inputParam = "sleeptime_s_str";
      sleeptime_s_str = inputMessage;
      Serial.printf("[%s]: sleeptime_s_str received: %s\n",__func__,sleeptime_s_str);
      if (strlen(sleeptime_s_str.c_str()) == 0)
      {
        sleeptime_s_str = old_sleeptime_s_str;
        Serial.printf("[%s]: sleeptime_s_str is blank, reusing the old value: %s\n",__func__,sleeptime_s_str);
      } else
      {
        sleeptime_s_str = inputMessage;
      }
      valid_sleeptime_s_str_received = true;
    }

    request->send(200, "text/html", "The values entered by you have been successfully sent to the device. It will now attempt WiFi connection");
  });
}

void WiFiSoftAPSetup()
{
  char cp_ssid[32];
  snprintf(cp_ssid,sizeof(cp_ssid),"%s",HOSTNAME);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(cp_ssid);
  Serial.printf("[%s]: AP IP address: ",__func__);
  Serial.println(WiFi.softAPIP());
  Serial.printf("[%s]: SSID: %s\n",__func__,cp_ssid);
  Serial.printf("[%s]: NO PASSWORD\n",__func__);
}

void WiFiStationSetup(String rec_ssid, String rec_password, String rec_channel, String rec_sleep_s)
{
  wifi_timeout = false;
  WiFi.mode(WIFI_STA);
  char ssid_arr[33];
  char password_arr[65];
  int channel;
  int sleeptime_s;
  rec_ssid.toCharArray(ssid_arr, rec_ssid.length() + 1);
  rec_password.toCharArray(password_arr, rec_password.length() + 1);

  channel = rec_channel.toInt();
  sleeptime_s = rec_sleep_s.toInt();

  #ifdef DEBUG
    Serial.printf("[%s]: Received SSID: %s, password: %s, channel: %d, sleeping time: %d \n",__func__,ssid_arr,password_arr, channel, sleeptime_s);
  #endif
  if ((channel > 0) and (channel < 15))
  {
     Serial.printf("[%s]: Valid fixed channel: %d\n", __func__,channel);
  } else
  {
    Serial.printf("[%s]: NOT valid fixed channel: %d\n", __func__,channel);
  }
  if ((sleeptime_s > 0) and (sleeptime_s <= 3600))
  {
     Serial.printf("[%s]: Valid sleeptime: %d seconds\n", __func__,sleeptime_s);
  } else
  {
    Serial.printf("[%s]: NOT valid sleeptime: %d, using default: %d seconds\n", __func__,sleeptime_s, (SLEEP_TIME_S));
  }
  WiFi.begin(ssid_arr, password_arr);
  Serial.printf("[%s]: Connecting...\n",__func__);
  uint32_t t1 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
    if (millis() - t1 > WAIT_FOR_WIFI * 1000)
    {
      Serial.println();
      Serial.printf("[%s]: Timeout connecting to WiFi. The SSID or Password seem incorrect\n",__func__);
      valid_ssid_received = false;
      valid_password_received = false;
      valid_channel_received = false;
      valid_sleeptime_s_str_received = false;
      is_setup_done = false;
      preferences.putBool("is_setup_done", is_setup_done);
      StartCaptivePortal();
      wifi_timeout = true;
      break;
    }
  }
  if (!wifi_timeout)
  {
    is_setup_done = true;
    Serial.printf("\n[%s]: WiFi connected to: %s\n",__func__,rec_ssid);
    valid_ssid_received = false;
    Serial.printf("[%s]: STA IP address: ",__func__);
    Serial.println(WiFi.localIP());
    preferences.putBool("is_setup_done", is_setup_done);
    preferences.putString("rec_ssid", rec_ssid);
    preferences.putString("rec_channel", rec_channel);
    preferences.putString("rec_password", rec_password);
    preferences.putString("rec_sleep_s", rec_sleep_s);
    if( led_blink_handle != NULL )
    {
      Serial.printf("[%s]: Disabling blinking LEDs\n",__func__);
      vTaskDelete( led_blink_handle );
      delay(5);
    } else
    {
      Serial.printf("[%s]: LEDs still bllinking or were never blinking\n",__func__);
    }
    Serial.printf("[%s]: Done\n",__func__);
  }
}

void StartCaptivePortal() {
  // blink LEDs in task, if not yet blinking
  if (led_blink_handle == NULL)
  {
    xReturned_led_blink = xTaskCreatePinnedToCore(led_blink, "led_blink", 2000, NULL, 1, &led_blink_handle, CONFIG_ARDUINO_RUNNING_CORE);
    if( xReturned_led_blink != pdPASS )
    {
      Serial.printf("[%s]: CANNOT create led_blink task\n",__func__);
    } else
    {
      #ifdef DEBUG
        Serial.printf("[%s]: Task led_blink created\n",__func__);
      #endif
    }
  } else
  // blink LEDs already blinking
  {
    Serial.printf("[%s]: Task led_blink ALREADY created\n",__func__);
  }

  // create CP timer if not yet created
  if (cp_timer_handle  == NULL)
  {
    cp_timer_handle = xTimerCreate("MyTimer", pdMS_TO_TICKS(CP_TIMEOUT_S * 1000), pdTRUE, ( void * )id, &cp_timer);
    if( xTimerStart(cp_timer_handle, 10 ) != pdPASS )
    {
      Serial.printf("[%s]: CP timer start error\n",__func__);
    } else
    {
      #ifdef DEBUG
        Serial.printf("[%s]: CP timer STARTED\n",__func__);
      #endif
    }
  } else
  // CP timer created so restart it
  {
    if( xTimerReset( cp_timer_handle, 0 ) != pdPASS )
    {
      Serial.printf("[%s]: CP timer was not reset\n",__func__);
    } else
    {
      Serial.printf("[%s]: CP timer RE-STARTED\n",__func__);
    }
  }

  Serial.printf("[%s]: Setting up AP Mode\n",__func__);
  WiFiSoftAPSetup();
  Serial.printf("[%s]: Setting up Async WebServer\n",__func__);
  setupServer();
  Serial.printf("[%s]: Starting DNS Server\n",__func__);
  dnsServer.start(53, "*", WiFi.softAPIP());
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);//only when requested from AP
  server.begin();
  dnsServer.processNextRequest();
  Serial.printf("[%s]: Captive Portal and AP created, timeout set to %ds\n",__func__,CP_TIMEOUT_S);
}

// CONNECT WIFI USING CAPTIVE PORTAL
void connect_wifi()
{
  preferences.begin("wifi-secrets", false);
  is_setup_done = preferences.getBool("is_setup_done", false);
  ssid = preferences.getString("rec_ssid", "Sample_SSID");
  channel = preferences.getString("rec_channel", "12");
  password = preferences.getString("rec_password", "abcdefgh");
  sleeptime_s_str = preferences.getString("rec_sleep_s", "5");

  if (!is_setup_done)
  {
    StartCaptivePortal();
  }
  else
  {
    Serial.printf("[%s]: Using saved SSID and PASS to attempt WiFi Connection...\n",__func__);
    Serial.printf("[%s]: SSID: ",__func__);Serial.println(ssid);
    Serial.printf("[%s]: PASS: ",__func__);Serial.println(password);
    WiFiStationSetup(ssid, password, channel, sleeptime_s_str);
  }

  while (!is_setup_done)
  {
    dnsServer.processNextRequest();
    delay(10);
    if (valid_ssid_received && valid_password_received)
    {
      Serial.printf("[%s]: Attempting WiFi Connection...\n",__func__);
      WiFiStationSetup(ssid, password, channel, sleeptime_s_str);
    }
  }
  Serial.printf("[%s]: Done\n",__func__);
}
// CAPTIVE PORTAL END

// blinking in rtos
void led_blink(void *pvParams)
{
  #if defined (ERROR_RED_LED_GPIO) // and defined (ERROR_RED_LED_GPIO)
    while (1) {
        digitalWrite(ERROR_RED_LED_GPIO,LOW);
        vTaskDelay(50/portTICK_RATE_MS);
        digitalWrite(ERROR_RED_LED_GPIO,HIGH);
        vTaskDelay(50/portTICK_RATE_MS);
    }
  #endif
}
// blinking in rtos END

// functions
// change MAC
void change_mac()
{
  snprintf(fake_mac, sizeof(fake_mac), "%02x%02x%02x%02x%02x%02x",FixedMACAddress[0], FixedMACAddress[1], FixedMACAddress[2], FixedMACAddress[3], FixedMACAddress[4], FixedMACAddress[5]);
  WiFi.mode(WIFI_STA);

  #ifdef DEBUG
    byte mac[6];
    WiFi.macAddress(mac);
    char mac1[18];
    snprintf(mac1, sizeof(mac1), "%02x:%02x:%02x:%02x:%02x:%02x",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("[%s]: Old MAC: %s\n",__func__,mac1);
  #endif

  if (esp_wifi_set_mac(WIFI_IF_STA, &FixedMACAddress[0]) == ESP_OK)
  {
    // #ifdef DEBUG
      Serial.printf("[%s]: Changing MAC SUCCESSFULL\n",__func__);
    // #endif
  }  else
  {
    Serial.printf("[%s]: Changing MAC FAILED\n",__func__);
  }
  #ifdef DEBUG
    WiFi.macAddress(mac);
    snprintf(mac1, sizeof(mac1), "%02x:%02x:%02x:%02x:%02x:%02x",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("[%s]: New MAC: %s\n",__func__,mac1);
    Serial.printf("[%s]: Firmware bin file location: ",__func__);
    Serial.printf("%s/%s/%s\n",UPDATE_FIRMWARE_HOST,fake_mac,FW_BIN_FILE);
  #endif
}
// change MAC END

// blink nicely - SOS on upgrade failure
void sos(int led)
{
  Serial.printf("[%s]: SOS...\n",__func__);
  #define DIT_MS 50;
  int dit = DIT_MS;
  int dah = 3 * dit;
  int inter_dit = dit;
  int inter_letter = 3 * dit;
  int inter_word = 7 * dit;

  digitalWrite(led,LOW);
  // delay(inter_word);

  // s
  for (byte i=0; i<3; i++)
  {
    digitalWrite(led,HIGH);
    delay(dit);
    digitalWrite(led,LOW);
    delay(dit);
  }
  delay(inter_letter);
  // 0
  for (byte i=0; i<3; i++)
  {
    digitalWrite(led,HIGH);
    delay(dah);
    digitalWrite(led,LOW);
    delay(dit);
  }
  delay(inter_letter);
  // s
  for (byte i=0; i<3; i++)
  {
    digitalWrite(led,HIGH);
    delay(dit);
    digitalWrite(led,LOW);
    delay(dit);
  }
  delay(inter_letter);
  Serial.printf("[%s]: SOS DONE\n",__func__);
}

// FW upgrade wrapper
void do_update()
{
  Serial.printf("[%s]: FW UPGRADE starting...\n",__func__);
  int update_firmware_status = -1;
  update_firmware_status=update_firmware_prepare();
  if (update_firmware_status == 0)
  {
    Serial.printf("[%s]: RESTARTING - FW update SUCCESSFULL\n\n",__func__);
    // blink slowly when FW upgrade successfull
    for (int i=0;i<3;i++)
    {
      #ifdef ACT_BLUE_LED_GPIO
        digitalWrite(ACT_BLUE_LED_GPIO,LOW);
        delay(100);
        digitalWrite(ACT_BLUE_LED_GPIO,HIGH);
        delay(30);
      #elif defined(ACT_BLUE_LED_GPIO)
        digitalWrite(ERROR_RED_LED_GPIO,LOW);
        delay(100);
        digitalWrite(ERROR_RED_LED_GPIO,HIGH);
        delay(30);
        // digitalWrite(ERROR_RED_LED_GPIO,LOW);
      #endif
    }

  } else
  {
    Serial.printf("[%s]: FW update failed - reason: %d\nRESTARTING - FW update failed\n\n",__func__,update_firmware_status);
    #ifdef ERROR_RED_LED_GPIO
      sos(ERROR_RED_LED_GPIO);
    #elif defined(ACT_BLUE_LED_GPIO)
      sos(ACT_BLUE_LED_GPIO);
    #endif
  }
  do_esp_restart();
}

// real upgrade
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
    // ...blink ACT_BLUE_LED_GPIO
    #ifdef ACT_BLUE_LED_GPIO
      if (blink_led_status) {
        blink_led_status=LOW;
        digitalWrite(ACT_BLUE_LED_GPIO,blink_led_status);
      } else {
        blink_led_status=HIGH;
        digitalWrite(ACT_BLUE_LED_GPIO,blink_led_status);
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
  char firmware_file[255];
  snprintf(firmware_file,sizeof(firmware_file),"%s/%s/%s",UPDATE_FIRMWARE_HOST,fake_mac,FW_BIN_FILE);

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

void hibernate(bool force, int final_sleeping_time_s) // force = true -> wake up only on timer, false->also on GPIO if defined (Motion or FW)
{
  esp_deep_sleep_disable_rom_logging();       // it does not display welcome message - shorter time to wake up
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,         ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC8M,         ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO,       ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);

  //the next 2 lines are not working with ext0 but ok with ext1
  esp_sleep_config_gpio_isolate();
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,   ESP_PD_OPTION_OFF);

  if (force) // used on error // used when battery too low - sleep for 1h to avoid battery depletion
  {
    // esp_sleep_enable_timer_wakeup(SLEEP_TIME_H_BATTERY_EMPTY * 3600 * uS_TO_S_FACTOR);
    // Serial.printf("[%s]: going to sleep for %d hour(s) to save battery\n",__func__,SLEEP_TIME_H_BATTERY_EMPTY);
    // Serial.printf("[%s]: Bye...\n========= E N D =========\n",__func__);
    // esp_deep_sleep_start();
    #ifdef ERROR_RED_LED_GPIO
      sos(ERROR_RED_LED_GPIO);
    #elif defined(ACT_BLUE_LED_GPIO)
      sos(ACT_BLUE_LED_GPIO);
    #endif
  }

  #ifdef DEBUG
    #ifdef MOTION_SENSOR_GPIO
      Serial.printf("[%s]: MOTION_SENSOR_GPIO (to wake up device)=%d\n",__func__,MOTION_SENSOR_GPIO);
    #endif
    #ifdef FW_UPGRADE_GPIO
      Serial.printf("[%s]: FW_UPGRADE_GPIO (to wake up device)=%d\n",__func__,FW_UPGRADE_GPIO);
    #endif
  #endif
  uint64_t bitmask_dec;

  // https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/ for ext0 and ext1 examples
  #ifdef MOTION_SENSOR_GPIO
                  // if (motion)
                  // // when motion detected we don't allow second wakup on motion - first cooling time only or FW update
                  // {
                  //   //send ESP to deep unconditional sleep for predefined time -  wake up on timer (cooling period)
                  //   esp_sleep_enable_timer_wakeup(sleeptime_s * uS_TO_S_FACTOR);
                  //   //... or on GPIO ext1 (FW update)
                  //   #ifdef FW_UPGRADE_GPIO //if FW_UPGRADE_GPIO  defined, wake up on it
                  //     bitmask_dec = pow(2,FW_UPGRADE_GPIO);
                  //     #ifdef DEBUG
                  //       Serial.printf("[%s]: bitmask_dec (FW_UPGRADE_GPIO) in dec=%ju\n",__func__,bitmask_dec);
                  //     #endif
                  //     // ESP32-C3
                  //     #if (BOARD_TYPE == 4)
                  //       {
                  //         esp_deep_sleep_enable_gpio_wakeup(bitmask_dec, ESP_GPIO_WAKEUP_GPIO_HIGH);
                  //       }
                  //     #else
                  //       {
                  //         esp_sleep_enable_ext1_wakeup(bitmask_dec, ESP_EXT1_WAKEUP_ANY_HIGH);
                  //       }
                  //     #endif

                  //   #endif

                  //   Serial.printf("[%s]: going to sleep for %d seconds  (cooling time)\n",__func__, sleeptime_s);
                  // } else
                  // {
                  //   //send ESP to deep unconditional sleep for predefined time -  wake up on timer...(heartbeat)
                  //   esp_sleep_enable_timer_wakeup(sleeptime_s * uS_TO_S_FACTOR);
                  //   //... or on GPIO ext1 (PIR motion detected) or FW_UPGRADE_GPIO
                  //   #ifdef FW_UPGRADE_GPIO //if FW_UPGRADE_GPIO  defined, wake up on it or on PIR - actually FW_UPGRADE_GPIO is obligatory
                  //     bitmask_dec = pow(2,FW_UPGRADE_GPIO) + pow(2,MOTION_SENSOR_GPIO);
                  //     #ifdef DEBUG
                  //       Serial.printf("[%s]: bitmask_dec (FW_UPGRADE_GPIO + MOTION_SENSOR_GPIO) in dec=%ju\n",__func__,bitmask_dec);
                  //     #endif
                  //   #else //if FW_UPGRADE_GPIO not defined, wake up only on PIR
                  //     bitmask_dec = pow(2,MOTION_SENSOR_GPIO);
                  //     #ifdef DEBUG
                  //       Serial.printf("[%s]: bitmask_dec=2^GPIO in dec=%d\n",__func__,bitmask_dec);
                  //     #endif
                  //   #endif
                  //   // ESP32-C3
                  //   #if (BOARD_TYPE == 4)
                  //     {
                  //       esp_deep_sleep_enable_gpio_wakeup(bitmask_dec, ESP_GPIO_WAKEUP_GPIO_HIGH);
                  //     }
                  //   #else
                  //     {
                  //       esp_sleep_enable_ext1_wakeup(bitmask_dec, ESP_EXT1_WAKEUP_ANY_HIGH);
                  //     }
                  //   #endif

                  //   Serial.printf("[%s]: going to sleep until next motion detected\n",__func__);
                  //   Serial.printf("[%s]:  or for %ds (heartbeat)\n",__func__,SLEEP_TIME_S);
                  // }
  #else
    //send ESP to deep unconditional sleep for predefined time -  wake up on timer...(heartbeat)
    esp_sleep_enable_timer_wakeup(final_sleeping_time_s * uS_TO_S_FACTOR);
    //... or on GPIO ext1 FW_UPGRADE_GPIO
    #ifdef FW_UPGRADE_GPIO //if FW_UPGRADE_GPIO  defined, wake up on it or on PIR - actually FW_UPGRADE_GPIO is obligatory
      bitmask_dec = pow(2,FW_UPGRADE_GPIO);
      #ifdef DEBUG
        Serial.printf("[%s]: bitmask_dec (FW_UPGRADE_GPIO) in dec=%ju\n",__func__,bitmask_dec);
      #endif
      // ESP32-C3
      #if (BOARD_TYPE == 4)
        {
          esp_deep_sleep_enable_gpio_wakeup(bitmask_dec, ESP_GPIO_WAKEUP_GPIO_HIGH);
        }
      #else
        {
          esp_sleep_enable_ext1_wakeup(bitmask_dec, ESP_EXT1_WAKEUP_ANY_HIGH);
        }
      #endif
    #endif
    Serial.printf("[%s]: going to sleep for %d seconds (heartbeat)\n",__func__, final_sleeping_time_s);
  #endif


    // testing with PPK2 - go to sleep
  #ifdef PPK2_GPIO
    digitalWrite(PPK2_GPIO,HIGH);
  #endif

  #ifdef ACT_BLUE_LED_GPIO
    digitalWrite(ACT_BLUE_LED_GPIO,LOW);
  #endif

  end_time = millis();
  if (boot_reason != 8) //timer
  {
    work_time = end_time - start_time + (ESP32_BOOT_TIME) + (ESP32_TAIL_TIME) + ESP32_BOOT_TIME_EXTRA;
  } else                // reset or power on
  {
    work_time = end_time - start_time + (ESP32_BOOT_TIME) + (ESP32_TAIL_TIME);
  }
  #ifdef DEBUG
    Serial.printf("[%s]: start_time=%ums, end_time=%ums, work_time=%dms\n",__func__,start_time,end_time,work_time);
  #else
    Serial.printf("[%s]: Program finished in %dms\n",__func__,work_time);
  #endif
  Serial.printf("========= E N D =========\n");
  Serial.flush();
  delay(1);
  esp_deep_sleep_start();
}

void do_esp_restart()
{
  Serial.printf("[%s]: RESTARTING...\n",__func__);
  // NOT GOOD IDEA TO SOS() WHEN "NORMAL" RESTART I.E. FW UPDATE
  // #ifdef ERROR_RED_LED_GPIO
  //   sos(ERROR_RED_LED_GPIO);
  // #elif defined(ACT_BLUE_LED_GPIO)
  //   sos(ACT_BLUE_LED_GPIO);
  // #endif
  #ifdef ERROR_RED_LED_GPIO
    digitalWrite(ERROR_RED_LED_GPIO,LOW);
    delay(10);
    digitalWrite(ERROR_RED_LED_GPIO,HIGH);
    delay(200);
  #elif defined(ACT_BLUE_LED_GPIO)
    digitalWrite(ACT_BLUE_LED_GPIO,LOW);
    delay(10);
    digitalWrite(ACT_BLUE_LED_GPIO,HIGH);
    delay(200);
  #endif
  ESP.restart();
}

void cp_timer( TimerHandle_t cp_timer_handle )
{
  Serial.printf("[%s]: Captive Portal timer expired\n",__func__);
  do_esp_restart();
}

void setup()
{
  start_time = millis();
  Serial.begin(115200);
  delay(1); //50
  Serial.printf("\n======= S T A R T =======\n");
  Serial.printf("%s, version: %s, compiled on: %s\n",HOSTNAME,FW_VERSION,COMPILATION_TIME);
  #ifdef DEBUG
    homeSpan.setLogLevel(2);
  #else
    homeSpan.setLogLevel(0);
  #endif
  #ifdef ACT_BLUE_LED_GPIO
    #ifdef DEBUG
      Serial.printf("[%s]: Configuring and turning ON ACT_BLUE_LED=%d\n",__func__,ACT_BLUE_LED_GPIO);
    #endif
      pinMode(ACT_BLUE_LED_GPIO,OUTPUT);
      digitalWrite(ACT_BLUE_LED_GPIO,HIGH);
  #endif
  #ifdef ERROR_RED_LED_GPIO
    #ifdef DEBUG
      Serial.printf("[%s]: Configuring and turning OFF ERROR_RED_LED=%d\n",__func__,ERROR_RED_LED_GPIO);
    #endif
      pinMode(ERROR_RED_LED_GPIO,OUTPUT);
      digitalWrite(ERROR_RED_LED_GPIO,LOW);
  #endif

  change_mac();

  #ifdef DEBUG
    display_wifi_credentials();
  #endif

  // get channel and sleeptime from preferences
  #ifdef DEBUG
    unsigned int start_pref = millis();
  #endif
  preferences.begin("wifi-secrets", true); // read only
  channel = preferences.getString("rec_channel", "0");
  sleeptime_s_str = preferences.getString("rec_sleep_s", "0");
  preferences.end();
  int wifi_channel = channel.toInt();
  sleeptime_s = sleeptime_s_str.toInt();

  // check sleeptime
  if ((sleeptime_s < 1) or (sleeptime_s > 3600))
  {
    // #ifdef DEBUG
      Serial.printf("[%s]: Found saved INVALID sleep time=%d seconds (it should be: 1-3600 seconds)\n",__func__,sleeptime_s);
    // #endif
    sleeptime_s = SLEEP_TIME_S; // anything between 1s to 3600s or default to SLEEP_TIME_S from config file
    preferences.begin("wifi-secrets", false);     // read/write
    preferences.putString("rec_sleep_s",String(sleeptime_s));
    preferences.end();
    #ifdef DEBUG
      Serial.printf("[%s]: Will use the default one=%d seconds\n",__func__,sleeptime_s);
      Serial.printf("[%s]: Saving default sleeptime(%d seconds) to preferences\n",__func__,sleeptime_s);
    #endif
  } else
  {
    #ifdef DEBUG
      Serial.printf("[%s]: Found saved sleep time=%d seconds - will use it\n",__func__,sleeptime_s);
    #endif
  }

  // check channel
  bool valid_wifi_channel = false;
  if ((wifi_channel > 0) and (wifi_channel < 14))
  {
    valid_wifi_channel = true;
    #ifdef DEBUG
      Serial.printf("[%s]: Found saved WiFi channel=%d - will use it for ESPnow at first\n",__func__,wifi_channel);
    #endif
  } else
  {
    // #ifdef DEBUG
      Serial.printf("[%s]: Found saved WiFi channel=%d is incorrect - will NOT use it for ESPnow!\n",__func__,wifi_channel);
    // #endif
  }

  #ifdef DEBUG
    unsigned int end_pref = millis();
    Serial.printf("[%s]: Time lost on preferences=%ums\n",__func__,(end_pref-start_pref));
  #endif
  // get channel and sleeptime from preferences END


  // turn ON power for I2C devices
  #ifdef ENABLE_3V_GPIO
    pinMode(ENABLE_3V_GPIO,OUTPUT);
    #ifdef DEBUG
      Serial.printf("[%s]: Enabling 3V to power sensors on GPIO=%d\n",__func__,ENABLE_3V_GPIO);
    #endif
    digitalWrite(ENABLE_3V_GPIO, HIGH);
  #endif
  delay(1);
  #if defined(CUSTOM_SDA_GPIO) and defined(CUSTOM_SCL_GPIO)
    #ifdef DEBUG
      Serial.printf("[%s]: CUSTOM I2C GPIO pins applied: SDA=%d, SCL=%d\n",__func__,CUSTOM_SDA_GPIO,CUSTOM_SCL_GPIO);
    #endif
    Wire.setPins(CUSTOM_SDA_GPIO,CUSTOM_SCL_GPIO);
  #endif
  Wire.begin();
  delay(1);

  boot_reason = esp_reset_reason();
  wakeup_reason = esp_sleep_get_wakeup_cause();

// MAX17048 - fuel gauge
#if (USE_MAX17048 == 1)
  unsigned long max17048_begin_time = millis();
  #ifdef DEBUG
    lipo.enableDebugging();
    Serial.printf("[%s]: start USE_MAX17048\n",__func__);
  #endif
  if (! lipo.begin())
  {
    Serial.printf("[%s]: MAX17048 NOT detected ... Check your wiring or I2C ADDR!\n",__func__);
    // not good idea
    // Serial.printf("[%s]: Going to sleep for %ds on ERROR!\n",__func__,(sleeptime_s / 2));
    // hibernate(true, (sleeptime_s / 2)); // sleep half time on error
  } else
  {
    // lipo.quickStart();     // not needed rather, MAX17048 can recalculate in the time
    #ifdef DEBUG
      Serial.printf("[%s]: start MAX17048 OK\n",__func__);
    #endif
    // reset MAX17048 if NOT woke up from deep sleep and apply MAX17048_DELAY_ON_RESET_MS
    if (boot_reason != 8)
    {
      // #ifdef DEBUG
        Serial.printf("[%s]: Resetting MAX17048 and applying delay for %dms\n",__func__,MAX17048_DELAY_ON_RESET_MS);
      // #endif
      lipo.reset();
      delay(MAX17048_DELAY_ON_RESET_MS);
    }
  }
#else
  #ifdef DEBUG
    Serial.printf("[%s]: DONT USE_MAX17048\n",__func__);
  #endif
#endif

  #ifdef DEBUG
    Serial.printf("[%s]: Boot   cause=%d - ",__func__,boot_reason);
  #endif
  switch(boot_reason)
  {
    // 1 = reset/power on
    case ESP_RST_POWERON:
    {
      #ifdef DEBUG
        Serial.printf("power on or reset\n");
      #endif
      #ifdef ACT_BLUE_LED_GPIO
        digitalWrite(ACT_BLUE_LED_GPIO, HIGH);
      #endif
      #ifdef ERROR_RED_LED_GPIO
        digitalWrite(ERROR_RED_LED_GPIO, HIGH);
      #endif
      break;
    }
    // 3 = Software reset via esp_restart
    case ESP_RST_SW:
    {
      #ifdef DEBUG
        Serial.printf("software reset via esp_restart\n");
      #endif
      #ifdef ACT_BLUE_LED_GPIO
        digitalWrite(ACT_BLUE_LED_GPIO, HIGH);
      #endif
      #ifdef ERROR_RED_LED_GPIO
        digitalWrite(ERROR_RED_LED_GPIO, HIGH);
      #endif
      break;
    }
    // 8 = deep sleep
    case ESP_RST_DEEPSLEEP:
    {
      #ifdef DEBUG
        Serial.printf("wake up from deep sleep \n");
      #endif
      break;
    }
    default:
    {
      #ifdef DEBUG
        Serial.printf("other boot cause=%d\n");
      #endif
      break;
    }
  }

  #ifdef DEBUG
    Serial.printf("[%s]: Wakeup cause=%d - ",__func__,wakeup_reason);
  #endif
  switch(wakeup_reason)
  {
    // 0 = not deep sleep
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    {
      #ifdef DEBUG
        Serial.printf("wake up was not caused by exit from deep sleep\n");
      #endif
      break;
    }
    // 2 = not in use
    case ESP_SLEEP_WAKEUP_EXT0:
    {
      #ifdef DEBUG
        Serial.printf("external signal using RTC_IO (motion detected)\n");
      #endif
      break;
    }
    // 3 = fw update (FW_UPGRADE_GPIO) or motion detected (MOTION_SENSOR_GPIO) - not for ESP32-C3!
    #if (BOARD_TYPE != 4)
      case ESP_SLEEP_WAKEUP_EXT1:
      {
        wakeup_gpio_mask = esp_sleep_get_ext1_wakeup_status();
        wakeup_gpio = log(wakeup_gpio_mask)/log(2);
        #ifdef DEBUG
          Serial.printf("external signal using GPIO=%d, GPIO_MASK=%ju\n",wakeup_gpio,wakeup_gpio_mask);
        #endif
        #ifdef ERROR_RED_LED_GPIO
          digitalWrite(ERROR_RED_LED_GPIO, HIGH);
        #endif
        #ifdef FW_UPGRADE_GPIO
          if (wakeup_gpio == FW_UPGRADE_GPIO)
          {
            fw_update = true;
            #ifdef DEBUG
              Serial.printf("[%s]: woke up on FW button pressed\n",__func__);
              Serial.printf("[%s]: debouncing for 100ms\n",__func__);
            #endif
            delay(100);
          }
        #endif
        #ifdef MOTION_SENSOR_GPIO
          if (wakeup_gpio == MOTION_SENSOR_GPIO)
          {
            motion = true;
          }
        #endif
        break;
      }
    #endif
    // 4 = wake up on timer (SLEEP_TIME_S or COOLING_SLEEP_DURATION_S)
    case ESP_SLEEP_WAKEUP_TIMER:
    {
      #ifdef DEBUG
        Serial.printf("timer (cooling or heartbeat)\n");
      #endif
      #ifdef ACT_BLUE_LED_GPIO
        digitalWrite(ACT_BLUE_LED_GPIO, HIGH);
      #endif
      break;
    }
    // 5 = not in use
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
    {
      #ifdef DEBUG
        Serial.printf("WAKEUP_TOUCHPAD\n");
      #endif
      break;
    }
    // 6 = not in use
    case ESP_SLEEP_WAKEUP_ULP:
    {
      #ifdef DEBUG
        Serial.printf("WAKEUP_ULP\n");
      #endif
      break;
    }
    // 7 = for ESP32-C3 - ESP_SLEEP_WAKEUP_GPIO,for others it is from light sleep only (ESP32, S2 and S3) so not programmed
    #if (BOARD_TYPE == 4)
      case ESP_SLEEP_WAKEUP_GPIO:
      {
        wakeup_gpio_mask = esp_sleep_get_gpio_wakeup_status();
        wakeup_gpio = log(wakeup_gpio_mask)/log(2);
        #ifdef DEBUG
          Serial.printf("external signal using GPIO=%d, GPIO_MASK=%ju\n",wakeup_gpio,wakeup_gpio_mask);
        #endif
        #ifdef ERROR_RED_LED_GPIO
          digitalWrite(ERROR_RED_LED_GPIO, HIGH);
        #endif
        #ifdef FW_UPGRADE_GPIO
          if (wakeup_gpio == FW_UPGRADE_GPIO)
          {
            fw_update = true;
            #ifdef DEBUG
              Serial.printf("[%s]: woke up on FW button pressed\n",__func__);
              Serial.printf("[%s]: debouncing for 100ms\n",__func__);
            #endif
            delay(100);
          }
        #endif
        #ifdef MOTION_SENSOR_GPIO
          if (wakeup_gpio == MOTION_SENSOR_GPIO)
          {
            motion = true;
          }
          break;
        #endif
        break;
      }
    #endif
    default:
    {
      #ifdef DEBUG
        Serial.printf("OTHER Wakeup cause\n");
      #endif
      break;
    }
  }


  // FW upgrade here:
  // 1 time: continue as normal
  // short, < 3s: continue as normal
  // 3-6s, restart
  // >6s: FW update
  #ifdef FW_UPGRADE_GPIO
    // RED LED ON, BLUE OFF if FW_UPGRADE_GPIO was pressed
    if (fw_update)
    {
      #ifdef ACT_BLUE_LED_GPIO
        digitalWrite(ACT_BLUE_LED_GPIO, LOW);
      #endif
      #ifdef ERROR_RED_LED_GPIO
        digitalWrite(ERROR_RED_LED_GPIO, HIGH);
      #endif
      // check if FW_UPGRADE_GPIO is still high after boot
      pinMode(FW_UPGRADE_GPIO,INPUT);
      if ((digitalRead(FW_UPGRADE_GPIO)) and (fw_update))
      {
        Serial.printf("[%s]: FW_UPGRADE_GPIO is still pressed after starting...\n",__func__);
        Serial.printf("[%s]: Waiting for 3s...\n",__func__);
        delay(3000);
        if (digitalRead(FW_UPGRADE_GPIO))
        {
          // first 3s done
          Serial.printf("[%s]: 3s passed\n",__func__);
          Serial.printf("[%s]: FW_UPGRADE_GPIO is still pressed after 3s...\n",__func__);

          #ifdef ERROR_RED_LED_GPIO
            digitalWrite(ERROR_RED_LED_GPIO, LOW);
          #endif
          delay(50);
          #ifdef ACT_BLUE_LED_GPIO
            digitalWrite(ACT_BLUE_LED_GPIO, HIGH);
          #endif
          fw_update = false;
          Serial.printf("[%s]: Waiting for 3s...\n",__func__);
          delay(3000);
          if (digitalRead(FW_UPGRADE_GPIO))
          {
            // second 3s done so 6s in total
            #ifdef ERROR_RED_LED_GPIO
              digitalWrite(ERROR_RED_LED_GPIO, HIGH);
            #endif
            delay(50);
            #ifdef ACT_BLUE_LED_GPIO
              digitalWrite(ACT_BLUE_LED_GPIO, LOW);
            #endif
            Serial.printf("[%s]: Long press > 6s - FW update about to happen now...\n",__func__);
            connect_wifi();
            do_update();
          } else
          {
            // press > 3s but <6s so restart
            #ifdef ACT_BLUE_LED_GPIO
              digitalWrite(ACT_BLUE_LED_GPIO, LOW);
            #endif
            Serial.printf("[%s]: Long press but <6s - Resetting WiFi credentials...\n",__func__);
            reset_wifi_credentials();
            connect_wifi();
            Serial.printf("[%s]: Restarting...\n",__func__);
            do_esp_restart();
          }
        } else // press < 3s (but pressed during ESP start) so nothing - continue
        {
          #ifdef DEBUG
            Serial.printf("[%s]: Short press (<3s) - continuing normal boot...\n",__func__);
          #endif
        }
      }
    }
    else    // FW_UPGRADE_GPIO was never pressed
    {}
  #endif

  // sht31
  // error and N/A values
  myData.md_temp= -99.9f;
  myData.md_hum = -99.9f;
  #if (USE_SHT31 == 1)
    #ifdef DEBUG
      unsigned sht31_start_time = millis();
    #endif
    if (sht31.begin(0x44))
    {
      #ifdef DEBUG
        Serial.printf("[%s]: Starting SHT31...SUCCESSFULL\n",__func__);
      #endif
      sht31.readBoth(&myData.md_temp,&myData.md_hum);       // 2 times faster than reading each separatelly, 22ms vs 44ms
    } else
    {
      Serial.printf("[%s]: Starting SHT31...FAILED\n",__func__);
      // not good idea
      // Serial.printf("[%s]: Going to sleep for %ds on ERROR!\n",__func__,(sleeptime_s / 2));
      // hibernate(true, (sleeptime_s / 2)); // sleep half time on error
    }
    #ifdef DEBUG
      unsigned sht31_end_time = millis();
      Serial.printf("[%s]: Measuring temp/hum took %ums\n",__func__,(sht31_end_time-sht31_start_time));
    #endif
  #else
    #ifdef DEBUG
      Serial.printf("[%s]: SHT31 NOT ENABLED\n",__func__);
    #endif
  #endif

  // tsl2561
  // error and N/A value
  myData.md_light = -99.9f;
  #if (USE_TSL2561 == 1)
    unsigned tsl_start_time = millis();
    if (! light.begin())
    {
      #ifdef DEBUG
         Serial.printf("[%s]: Starting TSL2561...FAILED\n",__func__);
      #endif
    }
    else
    {
      if (light.setPowerUp())
      {
        #ifdef DEBUG
          // Serial.printf("OK\n");
          Serial.printf("[%s]: Starting TSL2561...SUCCESSFULL\n",__func__);
        #endif
        unsigned int data0, data1; //data0=infrared, data1=visible light
        double lux;              // Resulting lux value
        boolean good;            // True if neither sensor is saturated
        boolean gain = false;     // Gain setting, 0 = X1, 1 = X16; if with gain and overshoots it goes from 0 again - better false
        unsigned int ms;         // Integration ("shutter") time in milliseconds
        // If time = 0, integration will be 13.7ms
        // If time = 1, integration will be 101ms
        // If time = 2, integration will be 402ms
        // If time = 3, use manual start / stop to perform your own integration
        unsigned char time = 0;
        light.begin();
        light.setTiming(gain,time,ms);
        light.manualStart();
        delay(ms);
        light.manualStop();
        if (light.getData(data0,data1))
        {
          good = light.getLux(gain,ms,data0,data1,lux);
          myData.md_light = lux + 0.0001f; // HomeKit does not accept 0.0000 as valid range
        }
      } else
      {
        // #ifdef DEBUG
          // Serial.printf("FAILED - cannot start\n");
          Serial.printf("[%s]: Starting TSL2561...FAILED - cannot power up\n",__func__);
          // not good idea
          // Serial.printf("[%s]: Going to sleep for %ds on ERROR!\n",__func__,(sleeptime_s / 2));
          // hibernate(true, (sleeptime_s / 2)); // sleep half time on error
        // #endif
      }
    }
    #ifdef DEBUG
      unsigned tsl_end_time = millis();
      Serial.printf("[%s]: Measuring light took %ums\n",__func__,(tsl_end_time-tsl_start_time));
    #endif
  #else
    #ifdef DEBUG
      Serial.printf("[%s]: TSL2561 NOT ENABLED\n",__func__);
    #endif
  #endif

myData.md_bat = 0;
float volts = 0.0f;
float bat_pct_float = 0.0f;
#if (USE_MAX17048 == 1)
  unsigned max17048_start_measure_time = millis();
  #ifdef DEBUG
    Serial.printf("[%s]: Time since start=%dms\n",__func__,(max17048_start_measure_time-max17048_begin_time));
  #endif
  volts = lipo.getVoltage();
  bat_pct_float = lipo.getSOC();
  lipo.getChangeRate();
  volts = lipo.getVoltage();
  bat_pct_float = lipo.getSOC();

  // Serial.printf("[%s]: Battery %% org=%0.2f%\n",__func__,bat_pct_float);
  u_int8_t bat_pct = round(bat_pct_float);
  if (bat_pct > 100) bat_pct=100;

  // Serial.printf("[%s]: Battery %% adjusted=%d%\n",__func__,bat_pct);

  myData.md_bat=bat_pct;

  // volts = 0.9;   // for testing only
  if (volts < 1)
  {
    long delay_time = (MAX17048_DELAY_ON_RESET_MS-(max17048_start_measure_time-max17048_begin_time));
    if (delay_time > 0)
    {
      Serial.printf("[%s]: Delaying to initialise MAX17048 for =%ums\n",__func__,delay_time);
      delay(delay_time);
    }
    volts = lipo.getVoltage();
  }
  #ifdef DEBUG
    Serial.printf("[%s]: Battery=%0.2fV\n",__func__,volts);
    Serial.printf("[%s]: Low battery threshold=%0.2fV, Minimum battery threshold=%0.2f\n",__func__,LOW_BATTERY_VOLTS,MINIMUM_VOLTS);
  #endif
  if (volts <= LOW_BATTERY_VOLTS)
  {
    if (volts <= MINIMUM_VOLTS)
    {
      Serial.printf("[%s]: Battery empty (%0.2fV) - going to sleep for %ds\n",__func__,volts,(24*60*60));
      hibernate(true, (SLEEP_TIME_H_BATTERY_EMPTY*60*60)); // 24 hours sleep if battery empty
    }
    #ifdef DEBUG
      Serial.printf("[%s]: LOW battery!\n",__func__);
    #endif
  } else
  {
    #ifdef DEBUG
      Serial.printf("[%s]: Battery OK\n",__func__);
    #endif
  }
  #ifdef DEBUG
    unsigned max17048_end_time = millis();
    Serial.printf("[%s]: Measuring battery took %ums\n",__func__,(max17048_end_time-max17048_start_measure_time));
  #endif
  // lipo.sleep();
#endif

  // turn OFF power for I2C devices
  #ifdef ENABLE_3V_GPIO
    #ifdef DEBUG
      Serial.printf("[%s]: Disabling 3V power for sensors on GPIO=%d\n",__func__,ENABLE_3V_GPIO);
      digitalWrite(ENABLE_3V_GPIO, LOW);
    #endif
  #endif

  // other info
  myData.md_mcu_model = BOARD_TYPE;
  snprintf(myData.md_version,sizeof(myData.md_version),"%s",FW_VERSION);

  Serial.printf("\n[%s]: Temperature=%0.2fC, Humidity=%0.2f%%, Light=%0.2flx, Battery percent=%d%%, Volts=%0.2fV\n",__func__,myData.md_temp,myData.md_hum,myData.md_light,myData.md_bat,volts);
  Serial.printf("[%s]: MCU model=%d, version=%s\n\n",__func__,myData.md_mcu_model,myData.md_version);

  // sending data to bridge
  #ifdef DEBUG
    Serial.printf("[%s]: Sending data to bridge...\n",__func__);
  #endif
  mainDevice=new SpanPoint(BRIDGE,sizeof(myData),0);    // create a SpanPoint with send size=sizeof(myData) and receive size=0
  // is ESPnow channel fixed?
/*
if valid channel:
{
  send,
  if successfull
  {
    bye
  } else
  {
    repeat sending for all channels from 1-14
    if successful
    {
      save
      bye
    } else
    {
      bye
    }
  }
} else // no channel provided
{
  sending for all channels from 1-14
  if successful
  {
    save
    bye
  } else
  {
    bye
  }
}
*/

  unsigned long start_send_data = millis();
  unsigned long end_send_data = 0;
  bool send_successful = false;
  if (valid_wifi_channel)
  {
    #ifdef DEBUG
      Serial.printf("[%s]: ESPnow configured to use only one channel=%d\n",__func__,wifi_channel);
    #endif
    SpanPoint::setChannelMask(1<<wifi_channel);
    if (mainDevice->send(&myData))      // first sending, either over 1 channel or over all channels
    {
      end_send_data = millis();
      Serial.printf("[%s]: Sending data to bridge - SUCCESSFULL, it took %ums\n",__func__,(end_send_data - start_send_data));
      send_successful = true;
    } else
    {
      Serial.printf("[%s]: Sending data to bridge - this time over all WiFi channels...\n",__func__);
      start_send_data = millis();
      for (int i=1;i<14;i++)
      {
        SpanPoint::setChannelMask(1<<i);
        if (mainDevice->send(&myData))
        {
          end_send_data = millis();
          send_successful = true;

          Serial.printf("[%s]: Sending data to bridge - SUCCESSFULL, it took %ums\n",__func__,(end_send_data - start_send_data));
          Serial.printf("[%s]: Setting and saving new WiFi channel (CHANNEL=%d) in preferences\n",__func__,i);

          preferences.begin("wifi-secrets", false);     // read/write
          preferences.putString("rec_channel",String(i));
          preferences.end();
          delay(10);
          break;
        }
      }
      if (!send_successful)
      {
        end_send_data = millis();
        Serial.printf("[%s]: Sending data to bridge - FAILED AGAIN (on ALL CHANNELS), it took %ums\n",__func__,(end_send_data - start_send_data));
      }
    }
  } else
  {
    Serial.printf("[%s]: Sending data to bridge over all WiFi channels...\n",__func__);
    start_send_data = millis();
    for (int i=1;i<14;i++)
    {
      SpanPoint::setChannelMask(1<<i);
      if (mainDevice->send(&myData))
      {
        end_send_data = millis();
        send_successful = true;

        Serial.printf("[%s]: Sending data to bridge - SUCCESSFULL, it took %ums\n",__func__,(end_send_data - start_send_data));
        Serial.printf("[%s]: Setting and saving new WiFi channel in preferences\n",__func__);

        preferences.begin("wifi-secrets", false);     // read/write
        preferences.putString("rec_channel",String(i));
        preferences.end();
        delay(10);
        break;
      }
    }
    if (!send_successful)
    {
      end_send_data = millis();
      Serial.printf("[%s]: Sending data to bridge - FAILED AGAIN (on ALL CHANNELS), it took %ums\n",__func__,(end_send_data - start_send_data));
    }
  }
  // sending data to bridge END

  hibernate(false, sleeptime_s);
  Serial.println("if you read this message, it means you lost your life ;-)");
}

void loop() {}
