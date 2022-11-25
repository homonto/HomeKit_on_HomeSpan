#if DEVICE_ID == 1
  #define HOSTNAME                    "homekit-sensor-1"
  #define BOARD_TYPE                  4     // 1 = ESP32-S, 2 = ESP32-S2, 3 = ESP32-S3, 4 = ESP32-C3

  #define ACT_BLUE_LED_GPIO           7     // comment out if not in use - don't use "0" here
  #define ERROR_RED_LED_GPIO          6     // comment out if not in use - don't use "0" here
  #define FW_UPGRADE_GPIO             5     // comment out if not in use - don't use "0" here <=5 for C3
  #define ENABLE_3V_GPIO              2     // comment out if not in use - don't use "0" here - mandatory for I2C devices on new boards
  #define CUSTOM_SDA_GPIO             18    // override default SDA pin
  #define CUSTOM_SCL_GPIO             19    // override default SCL pin
  // #define ADC_GPIO                    0     // comment out if not in use

  #define USE_MAX17048                1     // use "0" to disable
  #define USE_SHT31                   1     // use "0" to disable
  #define USE_TSL2561                 1     // use "0" to disable
  #define LOW_BATTERY_VOLTS           3.7   // low battery warning
  #define MINIMUM_VOLTS               3.3   // device is going to forced 12 hour sleep to conserve battery
  #define SLEEP_TIME_S                300    // seconds, 300 = 5min
  uint8_t FixedMACAddress[] =         {0x1a, 0x01, 0x01, 0x01, 0x01, 0x01};

  #pragma message "compilation for: DEVICE_ID=1, ESP32-C3, " HOSTNAME
// ---------------------------------------------------------------------------------------------------


#elif DEVICE_ID == 2
  #define HOSTNAME                    "homekit-sensor-2"
  #define BOARD_TYPE                  2     // 1 = ESP32-S, 2 = ESP32-S2, 3 = ESP32-S3, 4 = ESP32-C3

  #define ACT_BLUE_LED_GPIO           6     // comment out if not in use - don't use "0" here
  #define ERROR_RED_LED_GPIO          5     // comment out if not in use - don't use "0" here
  #define FW_UPGRADE_GPIO             4   // comment out if not in use - don't use "0" here - cannot be 8 or 9 on new boards if I2C used
  #define ENABLE_3V_GPIO              3     // comment out if not in use - don't use "0" here - mandatory for I2C devices on new boards
  #define CUSTOM_SDA_GPIO             8     // override default SDA pin
  #define CUSTOM_SCL_GPIO             9     // override default SCL pin
  #define POWER_GPIO                  38  // GREEN, STDB PIN6 on TP4056, LOW on CHARGED (LED ON),       comment out if not in use - don't use "0" here
  #define CHARGING_GPIO               39  // RED,   CHRG PIN7 on TP4056, LOW during CHARGING (LED ON),  comment out if not in use - don't use "0" here
  

  #define USE_MAX17048                1     // use "0" to disable
  #define USE_SHT31                   1     // use "0" to disable
  #define USE_TSL2561                 1     // use "0" to disable
  #define LOW_BATTERY_VOLTS           3.7   // low battery warning
  #define MINIMUM_VOLTS               3.3   // device is going to forced 12 hour sleep to conserve battery
  #define SLEEP_TIME_S                300    // seconds, 300 = 5min
  uint8_t FixedMACAddress[] =         {0x1a, 0x01, 0x01, 0x01, 0x01, 0x02};

  #pragma message "compilation for: DEVICE_ID=1, ESP32-S2, " HOSTNAME
// ---------------------------------------------------------------------------------------------------

#elif DEVICE_ID == 3
  #define HOSTNAME                    "homekit-sensor-3"
  #define BOARD_TYPE                  4     // 1 = ESP32-S, 2 = ESP32-S2, 3 = ESP32-S3, 4 = ESP32-C3

  #define ACT_BLUE_LED_GPIO           6     // comment out if not in use - don't use "0" here
  #define ERROR_RED_LED_GPIO          7     // comment out if not in use - don't use "0" here
  #define FW_UPGRADE_GPIO             5     // comment out if not in use - don't use "0" here <=5 for C3
  #define ENABLE_3V_GPIO              2     // comment out if not in use - don't use "0" here - mandatory for I2C devices on new boards
  #define CUSTOM_SDA_GPIO             18    // override default SDA pin
  #define CUSTOM_SCL_GPIO             19    // override default SCL pin
  #define POWER_GPIO                  1     // GREEN, STDB PIN6 on TP4056, LOW on CHARGED (LED ON),       comment out if not in use - don't use "0" here
  #define CHARGING_GPIO               0     // RED,   CHRG PIN7 on TP4056, LOW during CHARGING (LED ON),  comment out if not in use - don't use "0" here
  

  #define USE_MAX17048                1     // use "0" to disable
  #define USE_SHT31                   1     // use "0" to disable
  #define USE_TSL2561                 1     // use "0" to disable
  #define LOW_BATTERY_VOLTS           3.7   // low battery warning
  #define MINIMUM_VOLTS               3.3   // device is going to forced 12 hour sleep to conserve battery
  #define SLEEP_TIME_S                300    // seconds, 300 = 5min
  uint8_t FixedMACAddress[] =         {0x1a, 0x01, 0x01, 0x01, 0x01, 0x03};

  #pragma message "compilation for: DEVICE_ID=1, ESP32-C3, " HOSTNAME
// ---------------------------------------------------------------------------------------------------

#else
  #error "Wrong DEVICE_ID chosen"
#endif



// assigning MODEL and checking if proper board is selected
#ifndef BOARD_TYPE
  #error BOARD_TYPE not defined
#else
  #if (BOARD_TYPE == 1) and (!defined(CONFIG_IDF_TARGET_ESP32))
    #error wrong board selected in Arduino - choose ESP32DEV
  #endif
  #if (BOARD_TYPE == 2) and (!defined(CONFIG_IDF_TARGET_ESP32S2))
    #error wrong board selected in Arduino - choose S2
  #endif
  #if (BOARD_TYPE == 3) and (!defined(CONFIG_IDF_TARGET_ESP32S3))
    #error wrong board selected in Arduino - choose S3
  #endif
  #if (BOARD_TYPE == 4) and (!defined(CONFIG_IDF_TARGET_ESP32C3))
    #error wrong board selected in Arduino - choose C3
  #endif

  #if (BOARD_TYPE == 1)
    #define MODEL "ESP32"
    #define ESP32_BOOT_TIME           350      // since power ON to GPIO ON, ms
    #define ESP32_TAIL_TIME           15      // since GPIO OFF to power OFF, ms
    #define ESP32_BOOT_TIME_EXTRA     20     // +180ms from power ON or hard reset    #ifdef PRINT_COMPILER_MESSAGES
    #ifdef PRINT_COMPILER_MESSAGES
      #pragma message "chosen BOARD_TYPE = ESP32"
    #endif
  #elif (BOARD_TYPE == 2)
    #define MODEL "ESP32S2"
    // #define ESP32_IS_CHEATING (-228)  // -244 head, +12 tail, +4ms delay between saved and save_ontime() function start
    #define ESP32_BOOT_TIME           35      // since power ON to GPIO ON, ms
    #define ESP32_TAIL_TIME           25      // since GPIO OFF to power OFF, ms
    #define ESP32_BOOT_TIME_EXTRA     180     // +180ms from power ON or hard reset
    #ifdef PRINT_COMPILER_MESSAGES
      #pragma message "chosen BOARD_TYPE = ESP32S2"
    #endif
  #elif (BOARD_TYPE == 3)
    #define MODEL "ESP32S3"
    #define ESP32_IS_CHEATING 0       // not tested yet
    #ifdef PRINT_COMPILER_MESSAGES
      #pragma message "chosen BOARD_TYPE = ESP32S3"
    #endif
  #elif (BOARD_TYPE == 4)
    #define MODEL "ESP32C3"
    #define ESP32_BOOT_TIME           30      // since power ON to GPIO ON, ms
    #define ESP32_TAIL_TIME           10      // since GPIO OFF to power OFF, ms
    #define ESP32_BOOT_TIME_EXTRA     155     // +150ms from power ON or hard reset
    #ifdef PRINT_COMPILER_MESSAGES
      #pragma message "chosen BOARD_TYPE = ESP32C3"
    #endif
  #else
    #error BOARD_TYPE not defined
  #endif
#endif
// assigning MODEL and checking if proper board is selected END
