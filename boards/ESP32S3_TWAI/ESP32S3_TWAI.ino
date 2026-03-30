/*
    ESP32-S3 variant using the built-in TWAI (CAN) controller.
    No external MCP2515 needed — only a CAN transceiver (e.g. SN65HVD230).

    Default wiring:
      ESP32-S3       Transceiver (SN65HVD230)
      GPIO  4  -->   CTX (CAN TX)
      GPIO  5  <--   CRX (CAN RX)
      3.3V     -->   VCC
      GND      -->   GND

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "driver/twai.h"

// Compile-time target selection.
#define TARGET_LEGACY 1
#define TARGET_HW3 2
#define TARGET_HW4 3 // For pre-2026.2.3 HW4 firmware, compile as HW3.

#define HW_TARGET TARGET_HW3 // Change to TARGET_LEGACY, TARGET_HW3, or TARGET_HW4

bool enablePrint = true;

// ESP32-S3 TWAI pin configuration.
#define CAN_TX_PIN 4
#define CAN_RX_PIN 5
#define LED_PIN 2 // Built-in LED (some S3 boards use RGB on GPIO48 — adjust if needed)

// HW4 FSD V14 options
#define ENABLE_APPROACHING_EMERGENCY_VEHICLE_DETECTION true

// TWAI message wrapper to keep handler code consistent with MCP2515 variants.
struct can_frame {
  uint32_t can_id;
  uint8_t can_dlc;
  uint8_t data[8];
};

struct CarManagerBase {
  int speedProfile = 1;
  bool FSDEnabled = false;
  virtual void handleMessage(can_frame &frame) = 0;
};

inline uint8_t readMuxID(const can_frame &frame) {
  return frame.data[0] & 0x07;
}

inline bool isFSDSelectedInUI(const can_frame &frame) {
  return ((frame.data[4] >> 6) & 0x01) != 0;
}

inline void setSpeedProfileV12V13(can_frame &frame, int profile) {
  frame.data[6] &= ~0x06;
  frame.data[6] |= (profile << 1);
}

inline void setBit(can_frame &frame, int bit, bool value) {
  int byteIndex = bit / 8;
  int bitIndex = bit % 8;
  uint8_t mask = static_cast<uint8_t>(1U << bitIndex);

  if (value) {
    frame.data[byteIndex] |= mask;
  } else {
    frame.data[byteIndex] &= static_cast<uint8_t>(~mask);
  }
}

inline void sendFrame(can_frame &frame) {
  twai_message_t msg;
  msg.identifier = frame.can_id;
  msg.data_length_code = frame.can_dlc;
  msg.flags = 0;
  memcpy(msg.data, frame.data, 8);

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(10));
  if (enablePrint && result != ESP_OK) {
    Serial.printf("sendMessage failed: %d\n", result);
  }
}

struct LegacyHandler : public CarManagerBase {
  void handleMessage(can_frame &frame) override {
    // STW_ACTN_RQ (0x045 = 69): Follow-Distance stalk as speed profile source
    if (frame.can_id == 69) {
      uint8_t pos = frame.data[1] >> 5;
      if      (pos <= 1) speedProfile = 2;
      else if (pos == 2) speedProfile = 1;
      else               speedProfile = 0;
      return;
    }

    if (frame.can_id != 1006) {
      return;
    }

    uint8_t index = readMuxID(frame);
    if (index == 0) FSDEnabled = isFSDSelectedInUI(frame);

    if (index == 0 && FSDEnabled) {
      setBit(frame, 46, true);
      setSpeedProfileV12V13(frame, speedProfile);
      sendFrame(frame);
    }

    if (index == 1) {
      setBit(frame, 19, false);
      sendFrame(frame);
    }

    if (index == 0 && enablePrint) {
      Serial.printf("LegacyHandler: FSD: %d, Profile: %d\n", FSDEnabled, speedProfile);
    }
  }
};

struct HW3Handler : public CarManagerBase {
  int speedOffset = 0;

  void handleMessage(can_frame &frame) override {
    if (frame.can_id == 1016) {
      uint8_t followDistance = static_cast<uint8_t>((frame.data[5] & 0b11100000) >> 5);
      switch (followDistance) {
        case 1: speedProfile = 2; break;
        case 2: speedProfile = 1; break;
        case 3: speedProfile = 0; break;
        default: break;
      }
      return;
    }

    if (frame.can_id != 1021) {
      return;
    }

    uint8_t index = readMuxID(frame);
    bool selected = isFSDSelectedInUI(frame);

    if (index == 0 && selected) {
      int rawOff = static_cast<uint8_t>((frame.data[3] >> 1) & 0x3F) - 30;
      speedOffset = constrain(rawOff * 5, 0, 100);

      setBit(frame, 46, true);
      setSpeedProfileV12V13(frame, speedProfile);
      sendFrame(frame);
    }

    if (index == 1) {
      setBit(frame, 19, false);
      sendFrame(frame);
    }

    if (index == 2 && selected) {
      frame.data[0] &= ~(0b11000000);
      frame.data[1] &= ~(0b00111111);
      frame.data[0] |= static_cast<uint8_t>((speedOffset & 0x03) << 6);
      frame.data[1] |= static_cast<uint8_t>(speedOffset >> 2);
      sendFrame(frame);
    }

    if (index == 0 && enablePrint) {
      Serial.printf("HW3Handler: FSD: %d, Profile: %d, Offset: %d\n", selected, speedProfile, speedOffset);
    }
  }
};

struct HW4Handler : public CarManagerBase {
  void handleMessage(can_frame &frame) override {
    if (frame.can_id == 1016) {
      uint8_t fd = static_cast<uint8_t>((frame.data[5] & 0b11100000) >> 5);
      switch (fd) {
        case 1: speedProfile = 3; break;
        case 2: speedProfile = 2; break;
        case 3: speedProfile = 1; break;
        case 4: speedProfile = 0; break;
        case 5: speedProfile = 4; break;
        default: break;
      }
    }

    if (frame.can_id != 1021) {
      return;
    }

    uint8_t index = readMuxID(frame);
    bool selected = isFSDSelectedInUI(frame);

    if (index == 0 && selected) {
      setBit(frame, 46, true);
      setBit(frame, 60, true);
      if (ENABLE_APPROACHING_EMERGENCY_VEHICLE_DETECTION) {
        setBit(frame, 59, true);
      }
      sendFrame(frame);
    }

    if (index == 1) {
      setBit(frame, 19, false);
      setBit(frame, 47, true);
      sendFrame(frame);
    }

    if (index == 2) {
      frame.data[7] &= ~(0x07 << 4);
      frame.data[7] |= static_cast<uint8_t>((speedProfile & 0x07) << 4);
      sendFrame(frame);
    }

    if (index == 0 && enablePrint) {
      Serial.printf("HW4Handler: FSD: %d, Profile: %d\n", selected, speedProfile);
    }
  }
};

LegacyHandler legacyHandler;
HW3Handler hw3Handler;
HW4Handler hw4Handler;
CarManagerBase *handler = nullptr;

void setup() {
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);
  delay(1200);

#if HW_TARGET == TARGET_LEGACY
  handler = &legacyHandler;
#elif HW_TARGET == TARGET_HW3
  handler = &hw3Handler;
#elif HW_TARGET == TARGET_HW4
  handler = &hw4Handler;
#else
  #error Invalid HW selection. Use TARGET_LEGACY, TARGET_HW3, or TARGET_HW4.
#endif

  // Configure TWAI (CAN) peripheral at 500 kbit/s.
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("TWAI driver install failed");
    return;
  }

  if (twai_start() != ESP_OK) {
    Serial.println("TWAI start failed");
    return;
  }

  Serial.println("TWAI (CAN) ready @ 500k");
}

void loop() {
  twai_message_t rx;
  if (twai_receive(&rx, pdMS_TO_TICKS(1)) != ESP_OK) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  // Convert TWAI message to can_frame for handler compatibility.
  can_frame frame;
  frame.can_id = rx.identifier;
  frame.can_dlc = rx.data_length_code;
  memcpy(frame.data, rx.data, 8);

  digitalWrite(LED_PIN, LOW);
  if (handler != nullptr) {
    handler->handleMessage(frame);
  }
}
