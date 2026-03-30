/*
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

#include "memory"
#include <SPI.h>
#include <mcp2515.h>


// Compile-time target selection.
#define TARGET_LEGACY 1
#define TARGET_HW3 2
#define TARGET_HW4 3 // For pre-2026.2.3 HW4 firmware, compile as HW3.

#define HW_TARGET TARGET_HW3 // Change to TARGET_LEGACY, TARGET_HW3, or TARGET_HW4

bool enablePrint = true;


#define LED_PIN PIN_LED                // onboard red LED (GPIO13)
#define CAN_CS PIN_CAN_CS              // GPIO19 on this Feather
#define CAN_INT_PIN PIN_CAN_INTERRUPT  // GPIO22 (unused here; polling)
#define CAN_STBY PIN_CAN_STANDBY       // GPIO16
#define CAN_RESET PIN_CAN_RESET        // GPIO18

// HW4 FSD V14 options
#define ENABLE_APPROACHING_EMERGENCY_VEHICLE_DETECTION true

std::unique_ptr<MCP2515> mcp;

inline void sendFrame(can_frame& frame) {
  MCP2515::ERROR tx = mcp->sendMessage(&frame);
  if (enablePrint && tx != MCP2515::ERROR_OK) {
    Serial.printf("sendMessage failed: %d\n", static_cast<int>(tx));
  }
}

struct CarManagerBase {
  int speedProfile = 1;
  bool FSDEnabled = false;
  virtual void handleMessage(can_frame& frame) = 0;
};

inline uint8_t readMuxID(const can_frame& frame) {
  return frame.data[0] & 0x07;
}

inline bool isFSDSelectedInUI(const can_frame& frame) {
  return (frame.data[4] >> 6) & 0x01;
}

inline void setSpeedProfileV12V13(can_frame& frame, int profile) {
  frame.data[6] &= ~0x06;
  frame.data[6] |= (profile << 1);
}

inline void setBit(can_frame& frame, int bit, bool value) {
  // Determine which byte and which bit within that byte
  int byteIndex = bit / 8;
  int bitIndex = bit % 8;
  // Set the desired bit
  uint8_t mask = static_cast<uint8_t>(1U << bitIndex);
  if (value) {
    frame.data[byteIndex] |= mask;
  } else {
    frame.data[byteIndex] &= static_cast<uint8_t>(~mask);
  }
}


struct LegacyHandler : public CarManagerBase {
  virtual void handleMessage(can_frame& frame) override {
    // STW_ACTN_RQ (0x045 = 69): Follow-Distance stalk as speed profile source
    if (frame.can_id == 69) {
      uint8_t pos = frame.data[1] >> 5;
      if      (pos <= 1) speedProfile = 2;
      else if (pos == 2) speedProfile = 1;
      else               speedProfile = 0;
      return;
    }
    if (frame.can_id == 1006) {
      auto index = readMuxID(frame);
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
  }
};

struct HW3Handler : public CarManagerBase {
  int speedOffset = 0;
  virtual void handleMessage(can_frame& frame) override {
    if (frame.can_id == 1016) {
      uint8_t followDistance = (frame.data[5] & 0b11100000) >> 5;
      switch (followDistance) {
        case 1:
          speedProfile = 2;
          break;
        case 2:
          speedProfile = 1;
          break;
        case 3:
          speedProfile = 0;
          break;
        default:
          break;
      }
      return;
    }
    if (frame.can_id == 1021) {
      auto index = readMuxID(frame);
      bool selected = isFSDSelectedInUI(frame);
      if (index == 0 && selected) {
        int rawOff = static_cast<uint8_t>((frame.data[3] >> 1) & 0x3F) - 30;
        speedOffset = std::max(std::min(rawOff * 5, 100), 0);
        switch (rawOff) {
          case 2: speedProfile = 2; break;
          case 1: speedProfile = 1; break;
          case 0: speedProfile = 0; break;
          default: break;
        }
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
        frame.data[0] |= (speedOffset & 0x03) << 6;
        frame.data[1] |= (speedOffset >> 2);
        sendFrame(frame);
      }
      if (index == 0 && enablePrint) {
        Serial.printf("HW3Handler: FSD: %d, Profile: %d, Offset: %d\n", selected, speedProfile, speedOffset);
      }
    }
  }
};

struct HW4Handler : public CarManagerBase {
  virtual void handleMessage(can_frame& frame) override {
    if (frame.can_id == 1016) {
      auto fd = (frame.data[5] & 0b11100000) >> 5;
      switch(fd){
        case 1: speedProfile = 3; break;
        case 2: speedProfile = 2; break;
        case 3: speedProfile = 1; break;
        case 4: speedProfile = 0; break;
        case 5: speedProfile = 4; break;
      }
    }
    if (frame.can_id == 1021) {
      auto index = readMuxID(frame);
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
        frame.data[7] |= (speedProfile & 0x07) << 4;
        sendFrame(frame);
      }
      if (index == 0 && enablePrint) {
        Serial.printf("HW4Handler: FSD: %d, Profile: %d\n", selected, speedProfile);
      }
    }
  }
};


std::unique_ptr<CarManagerBase> handler;


void setup() {
#if HW_TARGET == TARGET_LEGACY
  handler = std::make_unique<LegacyHandler>();
#elif HW_TARGET == TARGET_HW3
  handler = std::make_unique<HW3Handler>();
#elif HW_TARGET == TARGET_HW4
  handler = std::make_unique<HW4Handler>();
#else
  #error Invalid HW selection. Use TARGET_LEGACY, TARGET_HW3, or TARGET_HW4.
#endif
  delay(1500);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1000) {}

  mcp = std::make_unique<MCP2515>(CAN_CS);

  mcp->reset();
  MCP2515::ERROR e = mcp->setBitrate(CAN_500KBPS, MCP_16MHZ);  
  if (e != MCP2515::ERROR_OK) Serial.println("setBitrate failed");
  mcp->setNormalMode();
  Serial.println("MCP25625 ready @ 500k");
}


void loop() {
  can_frame frame;
  int r = mcp->readMessage(&frame);
  if (r != MCP2515::ERROR_OK) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }
  digitalWrite(LED_PIN, LOW);
  handler->handleMessage(frame);
}
