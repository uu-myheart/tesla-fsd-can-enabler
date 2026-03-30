/*
    ESP32 + MCP2515 variant of the Tesla FSD CAN mod sketch.
    Uses an external MCP2515 CAN module connected via SPI.

    Default wiring (ESP32 VSPI):
      ESP32       MCP2515
      GPIO 18 --> SCK
      GPIO 19 <-- MISO (SO)
      GPIO 23 --> MOSI (SI)
      GPIO  5 --> CS
      GPIO  4 <-- INT (optional, polling used)

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

#include <SPI.h>
#include <mcp2515.h>

// Compile-time target selection.
#define TARGET_LEGACY 1
#define TARGET_HW3 2
#define TARGET_HW4 3 // For pre-2026.2.3 HW4 firmware, compile as HW3.

#define HW_TARGET TARGET_HW3 // Change to TARGET_LEGACY, TARGET_HW3, or TARGET_HW4

bool enablePrint = true;

// ESP32 + MCP2515 pin defaults.
#define LED_PIN 2
#define CAN_CS 5
#define CAN_INT_PIN 4 // Reserved for optional interrupt use.

// Most cheap MCP2515 modules use an 8MHz oscillator.
// Change this to MCP_16MHZ if your module has a 16MHz crystal.
#define MCP2515_CLOCK MCP_8MHZ

// HW4 FSD V14 options
#define ENABLE_APPROACHING_EMERGENCY_VEHICLE_DETECTION true

MCP2515 mcp(CAN_CS);

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
  MCP2515::ERROR tx = mcp.sendMessage(&frame);
  if (enablePrint && tx != MCP2515::ERROR_OK) {
    Serial.printf("sendMessage failed: %d\n", static_cast<int>(tx));
  }
}

struct LegacyHandler : public CarManagerBase {
  void handleMessage(can_frame &frame) override {
    if (frame.can_id != 1006) {
      return;
    }

    uint8_t index = readMuxID(frame);
    bool selected = isFSDSelectedInUI(frame);

    if (index == 0 && selected) {
      uint8_t off = static_cast<uint8_t>((frame.data[3] >> 1) & 0x3F) - 30;
      switch (off) {
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

    if (index == 0 && enablePrint) {
      Serial.printf("LegacyHandler: FSD: %d, Profile: %d\n", selected, speedProfile);
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
  pinMode(CAN_INT_PIN, INPUT_PULLUP);

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

  mcp.reset();
  MCP2515::ERROR e = mcp.setBitrate(CAN_500KBPS, MCP2515_CLOCK);
  if (e != MCP2515::ERROR_OK) {
    Serial.println("setBitrate failed");
  }

  mcp.setNormalMode();
  Serial.println("MCP2515 ready @ 500k");
}

void loop() {
  can_frame frame;
  int r = mcp.readMessage(&frame);

  if (r != MCP2515::ERROR_OK) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  digitalWrite(LED_PIN, LOW);
  if (handler != nullptr) {
    handler->handleMessage(frame);
  }
}
