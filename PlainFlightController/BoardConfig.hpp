#pragma once

#include <cstdint>
#include "Config.hpp"

// Create new board definitions here to match your use case.
// Note: Only one board type should be defined.
namespace BoardConfig {
#ifdef BOARD_XIAO_ESP32S3
  // Seeed Studio XIAO ESP32-S3
  // Output pins (servo/motor channels 1-6)
  static constexpr uint8_t OUTPUT_PIN_1  = 1U;   // GPIO1  = D0
  static constexpr uint8_t OUTPUT_PIN_2  = 2U;   // GPIO2  = D1
  static constexpr uint8_t OUTPUT_PIN_3  = 3U;   // GPIO3  = D2
  static constexpr uint8_t OUTPUT_PIN_4  = 4U;   // GPIO4  = D3
  static constexpr uint8_t OUTPUT_PIN_5  = 7U;   // GPIO7  = D8
  static constexpr uint8_t OUTPUT_PIN_6  = 8U;   // GPIO8  = D9

  //Auxillary IO pin allocation - only change if you know what you are doing
  static constexpr uint8_t LED_ONBOARD   = 21U;  // GPIO21 = LED_BUILTIN
  static constexpr uint8_t I2C_SDA       = 12U;  // GPIO12 not defined by arduino
  static constexpr uint8_t I2C_SCL       = 13U;  // GPIO13 not defined by arduino
  static constexpr uint8_t RECEIVER_RX   = 44U;  // GPIO44 = D7
  static constexpr uint8_t RECEIVER_TX   = 43U;  // GPIO43
  static constexpr uint8_t EXT_LED_PIN   = 43U;  // GPIO43 = D6
  static constexpr uint8_t BATT_ADC_PIN  = 9U;   // GPIO9  = D10
#elif defined(BOARD_WAVESHARE_ZERO)
  // Waveshare ESP32-S3 Zero
  // Output pins (servo/motor channels 1-6)
  static constexpr uint8_t OUTPUT_PIN_1  = 1U;   // GPIO1  = D6
  static constexpr uint8_t OUTPUT_PIN_2  = 2U;   // GPIO2  = D5
  static constexpr uint8_t OUTPUT_PIN_3  = 3U;   // GPIO3  = D4
  static constexpr uint8_t OUTPUT_PIN_4  = 4U;   // GPIO4  = D3
  static constexpr uint8_t OUTPUT_PIN_5  = 7U;   // GPIO7  = D2
  static constexpr uint8_t OUTPUT_PIN_6  = 8U;   // GPIO8  = D1

  //Auxillary IO pin allocation - only change if you know what you are doing
  static constexpr uint8_t LED_ONBOARD   = 21U;  // GPIO21
  static constexpr uint8_t I2C_SDA       = 12U;  // GPIO12 = OUTPUT_IO12
  static constexpr uint8_t I2C_SCL       = 13U;  // GPIO13 = OUTPUT_IO13
  static constexpr uint8_t EXT_LED_PIN   = 11U;  // GPIO11 = OUTPUT_IO11
  static constexpr uint8_t RECEIVER_RX   = 44U;  // GPIO44 = RX
  static constexpr uint8_t RECEIVER_TX   = 43U;  // GPIO43 = Tx
  static constexpr uint8_t BATT_ADC_PIN  = 9U;   // GPIO9  = OUTPUT_IO09
#else
  #error "No board defined. Please define a BOARD_xxx in Config.hpp"
#endif
}