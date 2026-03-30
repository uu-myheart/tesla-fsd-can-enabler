/*
    Feather M4 CAN Express (ATSAME51) variant of the Tesla FSD CAN mod sketch.
    Uses the ATSAME51's built-in CAN (MCAN) peripheral with the onboard
    TJA1051T/3 transceiver. No external MCP2515 needed.

    Required library: "Adafruit CAN" (CANSAME5x) via Arduino Library Manager.

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

#include <CANSAME5x.h>

// Compile-time target selection.
#define TARGET_LEGACY 1
#define TARGET_HW3 2
#define TARGET_HW4 3 // For pre-2026.2.3 HW4 firmware, compile as HW3.

#define HW_TARGET TARGET_HW3 // Change to TARGET_LEGACY, TARGET_HW3, or TARGET_HW4

bool enablePrint = true;

#define LED_PIN 13
#define CAN_STBY PIN_CAN_STANDBY       // CAN transceiver standby
#define CAN_BOOST PIN_CAN_BOOSTEN      // CAN boost enable (M4 CAN specific)

// HW4 FSD V14 options
#define ENABLE_APPROACHING_EMERGENCY_VEHICLE_DETECTION true

CANSAME5x CAN;

// Frame wrapper to keep handler code consistent with other board variants.
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
  CAN.beginPacket(frame.can_id, frame.can_dlc);
  CAN.write(frame.data, frame.can_dlc);
  bool ok = CAN.endPacket();
  if (enablePrint && !ok) {
    Serial.println("sendMessage failed");
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

  // Enable CAN transceiver boost (required on Feather M4 CAN)
  pinMode(CAN_BOOST, OUTPUT);
  digitalWrite(CAN_BOOST, HIGH);

  // Take transceiver out of standby
  pinMode(CAN_STBY, OUTPUT);
  digitalWrite(CAN_STBY, LOW);

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

  if (!CAN.begin(500000)) {
    Serial.println("CAN init failed");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }

  Serial.println("CANSAME5x ready @ 500k");
}

void loop() {
  int packetSize = CAN.parsePacket();
  if (packetSize <= 0) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  // Convert to can_frame for handler compatibility.
  can_frame frame;
  frame.can_id = CAN.packetId();
  frame.can_dlc = packetSize;
  for (int i = 0; i < packetSize && i < 8; i++) {
    frame.data[i] = CAN.read();
  }

  digitalWrite(LED_PIN, LOW);
  if (handler != nullptr) {
    handler->handleMessage(frame);
  }
}
