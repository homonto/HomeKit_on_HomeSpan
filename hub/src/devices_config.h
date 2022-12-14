
/*
ESP32-S2  is NOT OK, it has NOT ENOUGH memory to perform FW update (heap size < 64kB when HomeSpan is working)
ESP32S    is OK
ESP32-S3  is OK
ESP32-C3  not tested yet
*/



// #elif DEVICE_ID == 0                    // ESP32-S2 - not enough memory - don't use it - FW update fails - kept for GPIO and config example only
//     #define HOSTNAME                  "bridge1"
//     #define BOARD_TYPE                2     // 1 = ESP32-S, 2 = ESP32-S2, 3 = ESP32-S3, 4 = ESP32-C3
// ---------------------------------------------------------------------------------------------------
#if DEVICE_ID == 1                    // ESP32-S
    #define HOSTNAME                  "bridge1"
    #define BOARD_TYPE                1     // 1 = ESP32-S, 2 = ESP32-S2, 3 = ESP32-S3, 4 = ESP32-C3
    // bridge fake MAC
    uint8_t FixedMACAddress[] =       {0x1A, 0xFF, 0x01, 0x01, 0x01, 0x01};
    #define STATUS_LED_GPIO           33
    #define ERROR_RED_LED_GPIO        32
    #define CONTROL_PIN               23

    // with MAX17048
    #define CUSTOM_SDA_GPIO           21   // override default SDA pin
    #define CUSTOM_SCL_GPIO           22   // override default SCL pin
    #define USE_MAX17048              1   // use "0" to disable
    #define POWER_GPIO                25  // GREEN, STDB PIN6 on TP4056, LOW on CHARGED (LED ON),       comment out if not in use - don't use "0" here
    #define CHARGING_GPIO             26  // RED,   CHRG PIN7 on TP4056, LOW during CHARGING (LED ON),  comment out if not in use - don't use "0" here
    
  #pragma message "compilation for: DEVICE_ID=1, ESP32-S, " HOSTNAME
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
    // #define ESP32_BOOT_TIME           350      // since power ON to GPIO ON, ms
    // #define ESP32_TAIL_TIME           15      // since GPIO OFF to power OFF, ms
    // #define ESP32_BOOT_TIME_EXTRA     20     // +180ms from power ON or hard reset    #ifdef PRINT_COMPILER_MESSAGES
    #ifdef PRINT_COMPILER_MESSAGES
      #pragma message "chosen BOARD_TYPE = ESP32"
    #endif
  #elif (BOARD_TYPE == 2)
    #define MODEL "ESP32S2"
    // #define ESP32_BOOT_TIME           35      // since power ON to GPIO ON, ms
    // #define ESP32_TAIL_TIME           25      // since GPIO OFF to power OFF, ms
    // #define ESP32_BOOT_TIME_EXTRA     180     // +180ms from power ON or hard reset
    #ifdef PRINT_COMPILER_MESSAGES
      #pragma message "chosen BOARD_TYPE = ESP32S2"
    #endif
  #elif (BOARD_TYPE == 3)
    #define MODEL "ESP32S3"
    // #define ESP32_IS_CHEATING 0       // not tested yet
    #ifdef PRINT_COMPILER_MESSAGES
      #pragma message "chosen BOARD_TYPE = ESP32S3"
    #endif
  #elif (BOARD_TYPE == 4)
    #define MODEL "ESP32C3"
    // #define ESP32_BOOT_TIME           30      // since power ON to GPIO ON, ms
    // #define ESP32_TAIL_TIME           10      // since GPIO OFF to power OFF, ms
    // #define ESP32_BOOT_TIME_EXTRA     155     // +150ms from power ON or hard reset
    #ifdef PRINT_COMPILER_MESSAGES
      #pragma message "chosen BOARD_TYPE = ESP32C3"
    #endif
  #else
    #error BOARD_TYPE not defined
  #endif
#endif
// assigning MODEL and checking if proper board is selected END