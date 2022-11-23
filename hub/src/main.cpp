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

// #define DEVICE_ID           1 // ESP32S first bridge
// #define DEVICE_ID           2 // ESP32-S3 second bridge
#define DEVICE_ID           3 // ESP32-S2 third bridge

// #define DEBUG

#define OTA_ACTIVE                // OTA webserver

// libs
#include <Arduino.h>
#include "devices_config.h"
#include <esp_wifi.h>
#include "HomeSpan.h"
#include "passwords.h"
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

// MAX17048 - battery fuel gauge, I2C
#if (USE_MAX17048 == 1)
  #include <Wire.h>
  #include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
  SFE_MAX1704X lipo(MAX1704X_MAX17048);
#endif


// DEVICES
// sensors fake MAC addresses - must be the same on sensor devices!
#define DEVICE_ID_1               "1A:01:01:01:01:01" // C3
#define DEVICE_ID_1_NAME          "box1"
// #define DEVICE_ID_1_MODEL         "ESP32-C3"
#define DEVICE_ID_2               "1A:01:01:01:01:02" // S2 - without the box - development, office
#define DEVICE_ID_2_NAME          "box2"
// #define DEVICE_ID_2_MODEL         "ESP32-S2"
#define DEVICE_ID_3               "1A:01:01:01:01:03" // C3
#define DEVICE_ID_3_NAME          "box3"
// #define DEVICE_ID_3_MODEL         "ESP32-C3"

// devices FW - fake as it is updated by DEVICE when connected
#define DEVICE_FW                 "0.1.1"
// DEVICES END

// BRIDGE
// firmware:
#define BRIDGE_FW                 "0.1.8b1"
// BRIDGE END

// macros
#define SKETCH_VERSION            BRIDGE_FW 
#define MANUFACTURER              "PAPIO LTD"
#define NTP_SERVER                "pool.ntp.org"
#define NTP_SERVER_TIMEOUT_S      30
#define LOG_LEVEL                 0
#define UPDATE_TIMEOUT_S          900   // 900 = 15min, after this time Bridge reports failure of the device
//use one of them below: STATUS LED or ERROR LED
// #define BLINK_STATUS_LED_ON_RECEIVED_DATA   
#define BLINK_ERROR_LED_ON_RECEIVED_DATA    
#define BATTERY_INTERVAL_S        5   // in seconds, how often to update battery status and charging status of the hub on HomeKit (battery/charging is measured every second)
#define LOW_BATTERY_THRESHOLD     30  // in % to start complaining
#define IDENTIFY_BLINKS           10  // number of blinks when identify is called

// END of SETTING

typedef struct struct_message          // 28 bytes
{
  float md_temp;
  float md_hum;
  float md_light;
  byte md_low_bat;
  char md_version[10];
  byte md_mcu_model;
} struct_message;

struct_message myData;

// aux variables
float temperature_value;
float humidity_value;
float light_value;
byte low_bat_value        = 1;
char md_version_value[10];
byte md_mcu_model_value   = 0;
const char models[10][15]        = {"other", "ESP32", "ESP32-S2", "ESP32-S3", "ESP32-C3"}; //    // [number of models][length of string] - used only in Serial not on HomeKit

bool temperature_received = false;
bool humidity_received    = false;
bool light_received       = false;

bool temperature_timeout  = true;

bool device_timeout = false;

BaseType_t xReturned_check_volts;
TaskHandle_t check_volts_handle = NULL;
BaseType_t xReturned_led_blink;
TaskHandle_t led_blink_handle = NULL;

BaseType_t xReturned_check_charging;
TaskHandle_t check_charging_handle = NULL;

bool blinking = false;

float volts, bat_pct; 
// charging 
uint8_t charging_int = 0;          // 0-NC,  1-ON, 2-FULL,   // 3-off ??
const char charging_states[3][5] = {"NC", "ON", "FULL"};  // "OFF"};

// declarations
void led_blink(void *pvParams);
void check_volts(void*z);
void check_charging(void*z);


// blinking in rtos
void led_blink(void *pvParams) 
{
  #if defined (ERROR_RED_LED_GPIO) // and defined (ERROR_RED_LED_GPIO)
    while (1) {
        digitalWrite(ERROR_RED_LED_GPIO,HIGH);
        vTaskDelay(50/portTICK_RATE_MS);
        digitalWrite(ERROR_RED_LED_GPIO,LOW);
        vTaskDelay(50/portTICK_RATE_MS);
    }
  #endif
}
// blinking in rtos END


void check_volts(void*z)
{
  while(1)
    {
      volts   = lipo.getVoltage();
      bat_pct = lipo.getSOC();
      if (bat_pct>100) bat_pct=100; // we don't need crazy % here
      if (bat_pct<0)   bat_pct=0;

      LOG2("[%s]: Hub volts: %0.2f, battery percent=%0.2f%%\n",__func__,volts,bat_pct);
      if (bat_pct < LOW_BATTERY_THRESHOLD)
      {
         #ifdef DEBUG
          Serial.printf("[%s]: Volts too low: %0.2f, battery percent=%0.2f%%\n",__func__,volts,bat_pct);
        #endif
        // blink LEDs in task, if not yet blinking
        if (!blinking)
        {
          xReturned_led_blink = xTaskCreatePinnedToCore(led_blink, "led_blink", 5000, NULL, 1, &led_blink_handle, CONFIG_ARDUINO_RUNNING_CORE);
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

      vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// check charging
void check_charging(void*z)
{
  while(1)
  {
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

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


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
      temperature_value     = myData.md_temp;     // update temperature
      temperature_received  = true;
      temperature_timeout   = false;
      humidity_value        = myData.md_hum;      // update humidity
      humidity_received     = true;
      light_value           = myData.md_light;    // update light
      light_received        = true;
      low_bat_value         = myData.md_low_bat;  // update low battery
      md_mcu_model_value    = myData.md_mcu_model;  // update model

      snprintf(md_version_value,sizeof(md_version_value),"%s",myData.md_version); // update version
      
      LOG0("[%s]:  Raw data from device %s just arrived: temp=%0.2fC, hum=%0.2f%%, light=%0.2flx, low_bat=%d, model=%d (%s), version=%s\n",__func__,name,temperature_value,humidity_value,light_value,low_bat_value,md_mcu_model_value,models[md_mcu_model_value],md_version_value);

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
  SpanCharacteristic *low_bat;
  
  const char *name;
  const char *macAddress;

  RemoteTempSensor(const char *name, const char*macAddress) : Service::TemperatureSensor()
  {
    this->name=name;
    this->macAddress=macAddress;
    temp=new Characteristic::CurrentTemperature(0);           // set initial temperature
    temp->setRange(-50,150);                                  // expand temperature range to allow negative values
    fault=new Characteristic::StatusFault(0);                 // set initial state = no fault
    low_bat=new Characteristic::StatusLowBattery(1);          // initial battery status, 1=low battery, 0=battery OK
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
      // battery, together with temperature - no need for extra bool
      low_bat->setVal(low_bat_value);                      // update low_bat
      LOG1("[%s]: Low battery from sensor %s on device %s=%d\n",__func__,name,macAddress,low_bat_value);

      Serial.printf("[%s]: Device %s, sensor %s,temperature=%0.2fC, fault=%d\n",__func__,macAddress,name,temperature_value,fault->getVal());
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
      fault->setVal(1);   
      device_timeout = false; // clear timeout as all data already done for this device and this timeout is not relevant to next devices
    }
  }

};


struct UpdateBattery : Service::BatteryService
{
  SpanCharacteristic *battery_level;
  SpanCharacteristic *charging_state;
  SpanCharacteristic *low_battery;

  UpdateBattery() : Service::BatteryService()
  {
    new Characteristic::Name("Battery sensor");
    battery_level     = new Characteristic::BatteryLevel(0);                     // set initial state to 0%
    charging_state    = new Characteristic::ChargingState(0);                    // set initial state to "not charging"
    low_battery       = new Characteristic::StatusLowBattery(1);                 // set initial state to true
  }

  void loop()
  {
    
    if (battery_level->timeVal()>(BATTERY_INTERVAL_S * 1000))
    {
      LOG1("[%s]: Hub battery status: volts=%0.2fV, percent=%0.2f%%, charging=%d(%s)\n",__func__,volts,bat_pct,charging_int,charging_states[charging_int]);

      battery_level->setVal(bat_pct);  
      if ((charging_int == 1) or (charging_int == 2))
      {
        charging_state->setVal(1);
      }
      else 
      {
        charging_state->setVal(0);
      }
      
      if (bat_pct < LOW_BATTERY_THRESHOLD)
      {
        low_battery->setVal(1);  
        LOG0("[%s]: Hub battery CRITICAL status: volts=%0.2fV, percent=%0.2f%%, charging=%d(%s)\n",__func__,volts,bat_pct,charging_int,charging_states[charging_int]);
      } else 
      {
        low_battery->setVal(0);  
      }
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

    // pinMode(homeSpan.getStatusPin(),OUTPUT);          // make sure LED is set for output
  }

  boolean update()
  { 
    #ifdef ERROR_RED_LED_GPIO    
        LOG0("[%s]: hub called to identify itself, blinking ERROR LED...\n",__func__);
        for(int i=0;i<nBlinks;i++)
        {
            digitalWrite(ERROR_RED_LED_GPIO,HIGH);
            delay(50);
            digitalWrite(ERROR_RED_LED_GPIO,LOW);
            delay(50);
        }
    #elif defined(STATUS_LED_GPIO)
        LOG0("[%s]: hub called to identify, blinking status LED...\n",__func__);
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
    LOG0("[%s]: hub identified\n",__func__);
    return true;
  }

};

void setup() 
{
  Serial.begin(115200);
  Serial.printf("\n======= S T A R T =======\n");

  #ifdef ERROR_RED_LED_GPIO
    pinMode(ERROR_RED_LED_GPIO,OUTPUT);
  #endif

  homeSpan.setApSSID(HOSTNAME);
  homeSpan.setApPassword("");   // no password for CP
  homeSpan.enableOTA();         // homespan-ota
  homeSpan.enableAutoStartAP(); // AP if no WiFi credentials
  homeSpan.setHostNameSuffix("");;
  // homeSpan.setStatusAutoOff(5);  // turn OFF LED - not good as no info about the life of hub

  // change MAC
  char mac_org_char[22];
  byte mac_org[6];
  char mac_new_char[22];
  byte mac_new[6];

  WiFi.mode(WIFI_STA);
  WiFi.macAddress(mac_org);

  snprintf(mac_org_char, sizeof(mac_org_char), "%02x:%02x:%02x:%02x:%02x:%02x",mac_org[0], mac_org[1], mac_org[2], mac_org[3], mac_org[4], mac_org[5]);
  Serial.printf("[%s]: OLD MAC: %s\n",__func__,mac_org_char);

  Serial.printf("[%s]: changing MAC...",__func__);
  if (esp_wifi_set_mac(WIFI_IF_STA, &FixedMACAddress[0]) == ESP_OK) Serial.println("SUCCESSFULL"); else Serial.println("FAILED");

  WiFi.macAddress(mac_new);
  snprintf(mac_new_char, sizeof(mac_new_char), "%02x:%02x:%02x:%02x:%02x:%02x",mac_new[0], mac_new[1], mac_new[2], mac_new[3], mac_new[4], mac_new[5]);
  Serial.printf("[%s]: NEW MAC: %s\n",__func__,mac_new_char);
  // change MAC END

  // start checking charging
  #if defined(CHARGING_GPIO) and defined(POWER_GPIO)
    #ifdef DEBUG
      Serial.printf("[%s]: CHARGING_GPIO and POWER_GPIO enabled\n",__func__);
    #endif
    pinMode(CHARGING_GPIO, INPUT_PULLUP);  //both down: NC initially, will be changed when checked
    pinMode(POWER_GPIO, INPUT_PULLUP);
    xReturned_check_charging = xTaskCreatePinnedToCore(check_charging, "check_charging", 5000, NULL, 1, &check_charging_handle, CONFIG_ARDUINO_RUNNING_CORE);
    if( xReturned_check_charging != pdPASS )
    {
      Serial.printf("[%s]: check_charging task not created - restarting in 1s\n",__func__);
    } else
    {
      Serial.printf("[%s]: check_charging task created\n",__func__);
    }
  #else
    #ifdef DEBUG
      Serial.printf("[%s]: checking CHARGING DISABLED\n",__func__);
    #endif
  #endif
  // start start checking charging END  

  // MAX17048 - fuel gauge
  #if (USE_MAX17048 == 1)
    #if defined(CUSTOM_SDA_GPIO) and defined(CUSTOM_SCL_GPIO)
    #ifdef DEBUG
        Serial.printf("[%s]: CUSTOM I2C GPIO pins applied: SDA=%d, SCL=%d\n",__func__,CUSTOM_SDA_GPIO,CUSTOM_SCL_GPIO);
    #endif
      Wire.setPins(CUSTOM_SDA_GPIO,CUSTOM_SCL_GPIO);
    #endif
    Wire.begin();
    delay(100); 
    unsigned long max17048_begin_time = millis();
    #ifdef DEBUG
      lipo.enableDebugging();
      Serial.printf("[%s]: start USE_MAX17048\n",__func__);
    #endif
    if (! lipo.begin())
    {
      Serial.printf("[%s]: MAX17048 NOT detected ... Check your wiring or I2C ADDR!\n",__func__);
      Serial.printf("[%s]: HALTING PROGRAM\n",__func__);
      while (1){}
    } else
    {
      // lipo.quickStart();
      #ifdef DEBUG
        Serial.printf("[%s]: start MAX17048 OK\n",__func__);
      #endif
    }
    xReturned_check_volts = xTaskCreatePinnedToCore(check_volts, "check_volts", 5000, NULL, 1, &check_volts_handle, CONFIG_ARDUINO_RUNNING_CORE);
    if( xReturned_check_volts != pdPASS )
    {
      Serial.printf("[%s]: check_volts task not created - restarting in 1s\n",__func__);
    } else
    {
      Serial.printf("[%s]: check_volts task created\n",__func__);
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
  homeSpan.enableWebLog(50,NTP_SERVER,"UTC","stats");   // opens webpage with logs
  homeSpan.setTimeServerTimeout(NTP_SERVER_TIMEOUT_S);
  homeSpan.setSketchVersion(SKETCH_VERSION);

// HUB:
  new SpanAccessory();
    new DEV_Identify(HOSTNAME,MANUFACTURER,ORG_BRIDGE_MAC,MODEL,BRIDGE_FW,IDENTIFY_BLINKS);
    new UpdateBattery();
      
      

// DEVICE 1
  new SpanAccessory();
    new UpdateData(DEVICE_ID_1_NAME,DEVICE_ID_1);                 
    new RemoteTempSensor("Temperature Sensor",DEVICE_ID_1);        
    new RemoteHumSensor("Humidity Sensor",DEVICE_ID_1);        
    new RemoteLightSensor("Light Sensor",DEVICE_ID_1);  

// DEVICE 2
  new SpanAccessory();
    new UpdateData(DEVICE_ID_2_NAME,DEVICE_ID_2);                 
    new RemoteTempSensor("Temperature Sensor",DEVICE_ID_2);        
    new RemoteHumSensor("Humidity Sensor",DEVICE_ID_2);        
    new RemoteLightSensor("Light Sensor",DEVICE_ID_2);        

// DEVICE 3
  new SpanAccessory();
    new UpdateData(DEVICE_ID_3_NAME,DEVICE_ID_3);                 
    new RemoteTempSensor("Temperature Sensor",DEVICE_ID_3);        
    new RemoteHumSensor("Humidity Sensor",DEVICE_ID_3);        
    new RemoteLightSensor("Light Sensor",DEVICE_ID_3);  

  homeSpan.begin(Category::Bridges,HOSTNAME,HOSTNAME);

  //OTA in Setup
  #ifdef OTA_ACTIVE
    strlcpy(ota_user, OTA_USER, sizeof(ota_user));
    strlcpy(ota_password, OTA_PASSWORD, sizeof(ota_password));
    Serial.printf("[%s]: Enabling OTA:...\n",__func__);
    Serial.printf("[%s]: user=%s, password=%s\n",__func__,ota_user,ota_password);
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

}



void loop()
{
  homeSpan.poll();
}