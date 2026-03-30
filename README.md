# Tesla FSD CAN Bus Enabler

> **Background:** This project was inspired by [@mikegapinski](https://x.com/mikegapinski) and his [Tesla Android Diagnostic Tool](https://teslaandroid.com/products/tesla-diagnostic-tool), which enables FSD in regions where it is not yet fully available. I was interested in his product but was put off by the long delivery time and cost. Around the same time I came across a GitLab repository by Starmixcraft that took a simpler, DIY approach. The original repo appeared to be deleted but has since moved to [gitlab.com/Tesla-OPEN-CAN-MOD/tesla-fsd-can-mod](https://gitlab.com/Tesla-OPEN-CAN-MOD/tesla-fsd-can-mod) (the old Starmixcraft URL now redirects there). I picked up the last available version of that project and have been building on it — adding support for more boards and working toward feature parity with commercial solutions. The goal is not to undermine @mikegapinski's work (if you want a polished, plug-and-play product, go buy his tool), but to keep a DIY alternative available for the community.

## Project Structure

```
boards/
├── ESP32_MCP2515/       # ESP32 + external MCP2515 SPI module
│   └── ESP32_MCP2515.ino
├── ESP32S3_TWAI/        # ESP32-S3 with built-in TWAI (CAN) + transceiver (e.g. SN65HVD230)
│   └── ESP32S3_TWAI.ino
├── FeatherM4CAN/        # Adafruit Feather M4 CAN Express (ATSAME51 + MCP25625)
│   └── FeatherM4CAN.ino
├── RP2040CAN/           # Adafruit Feather RP2040 CAN (MCP25625)
│   └── RP2040CAN.ino
└── UNO_MCP2515_CAN/     # Arduino UNO + MCP2515 module
    └── UNO_MCP2515_CAN.ino
```

Each board variant lives in its own directory (required by the Arduino IDE). Pick the sketch that matches your hardware.

## 📌 Prerequisites

**You must have an active FSD package on the vehicle** — either purchased or subscribed. This board enables the FSD functionality on the CAN bus level, but the vehicle still needs a valid FSD entitlement from Tesla.

If FSD subscriptions are not available in your region, you can work around this by:

1. Creating a Tesla account in a region where FSD subscriptions are offered (e.g. Canada).
2. Transferring the vehicle to that account.
3. Subscribing to FSD through that account.

This allows you to activate an FSD subscription from anywhere in the world.

## 🛠️ What It Does

This firmware intercepts specific CAN bus messages to enable and configure Full Self-Driving (FSD). It supports multiple boards — from Adafruit Feathers with MCP25625, to ESP32 with external MCP2515 modules, to ESP32-S3 using its built-in TWAI (CAN) controller. Additionally, ASS (Actually Smart Summon) is no longer restricted by EU regulations.

🚗 Core Function

- Intercepts specific CAN bus messages
- Re-transmits them onto the vehicle bus

🧠 FSD Activation Logic

- Listens for Autopilot-related CAN frames
- Checks if "Traffic Light and Stop Sign Control" is enabled in the Autopilot settings Uses this setting as a trigger for Full Self-Driving (FSD)
- Adjusts the required bits in the CAN message to activate FSD

⚙️ Additional Behavior

- Reads the follow-distance stalk setting
- Maps it dynamically to a speed profile

⚙️ HW4 - FSD V14 Features

- Approaching Emergency Vehicle Detection

### Supported Hardware Variants

Select your hardware variant via the `#define HW` directive in the sketch for your board:

| Define   | Target       | Listens on CAN IDs | Notes                                         |
| -------- | ------------ | ------------------ | --------------------------------------------- |
| `LEGACY` | HW3 Retrofit | 69, 1006            | Sets FSD enable bit and speed profile via follow-distance stalk |
| `HW3`    | HW3 vehicles | 1016, 1021         | Adds speed-offset control via follow-distance |
| `HW4`    | HW4 vehicles | 1016, 1021         | Extended speed-profile range (5 levels)       |

> **Note:** HW4 vehicles on firmware **2026.2.9.X** are on **FSD v14**. However, versions on the **2026.8.X** branch are still on **FSD v13**. If your vehicle is running FSD v13 (including the 2026.8.X branch or anything older than 2026.2.9), compile with `HW3` even if your vehicle has HW4 hardware.

### How to Determine Your Hardware Variant

- **Legacy** — Your vehicle has a **portrait-oriented center screen** and **HW3**. This applies to older Model S and Model X vehicles retrofitted with HW3.
- **HW3** — Your vehicle has a **landscape-oriented center screen** and **HW3**. You can check your hardware version under **Controls → Software → Additional Vehicle Information** on the vehicle's touchscreen.
- **HW4** — Same as above, but the Additional Vehicle Information screen shows **HW4**.

### Key Behaviour

- **FSD enable bit** is set when **"Traffic Light and Stop Sign Control"** is enabled in the vehicle's Autopilot settings.
- **Speed profile** is derived from the scroll-wheel offset or follow-distance setting.
- **Nag suppression** — clears the hands-on-wheel nag bit.
- Debug output is printed over Serial at 115200 baud when `enablePrint` is `true`.

### CAN Message Details

The table below shows exactly which CAN messages each hardware variant monitors and what modifications are made.

#### Legacy (HW3 Retrofit)

| CAN ID | Hex | Name | Direction | Mux | Action |
|---|---|---|---|---|---|
| 69 | 0x045 | STW_ACTN_RQ | Read only | — | Read follow-distance stalk position → map to speed profile |
| 1006 | 0x3EE | — | Read + Modify | 0 | Read FSD state from UI; set bit 46 (FSD enable); write speed profile to bits 1–2 of byte 6 |
| 1006 | 0x3EE | — | Read + Modify | 1 | Clear bit 19 (nag suppression) |

#### HW3

| CAN ID | Hex | Name | Direction | Mux | Action |
|---|---|---|---|---|---|
| 1016 | 0x3F8 | — | Read only | — | Read follow-distance setting → map to speed profile |
| 1021 | 0x3FD | — | Read + Modify | 0 | Read FSD state from UI; calculate speed offset; set bit 46 (FSD enable); write speed profile to bits 1–2 of byte 6 |
| 1021 | 0x3FD | — | Read + Modify | 1 | Clear bit 19 (nag suppression) |
| 1021 | 0x3FD | — | Read + Modify | 2 | Write speed offset to bits 6–7 of byte 0 and bits 0–5 of byte 1 |

#### HW4

| CAN ID | Hex | Name | Direction | Mux | Action |
|---|---|---|---|---|---|
| 1016 | 0x3F8 | — | Read only | — | Read follow-distance setting → map to speed profile (5 levels) |
| 1021 | 0x3FD | — | Read + Modify | 0 | Read FSD state from UI; set bit 46 (FSD enable); set bit 60 (FSD V14); set bit 59 (emergency vehicle detection) |
| 1021 | 0x3FD | — | Read + Modify | 1 | Clear bit 19 (nag suppression); set bit 47 |
| 1021 | 0x3FD | — | Read + Modify | 2 | Write speed profile to bits 4–6 of byte 7 |

## Hardware Requirements

One of the following boards (or a compatible alternative):

| Board | CAN Interface | Notes |
|---|---|---|
| Adafruit Feather RP2040 CAN | MCP25625 (onboard) | Uses board-defined `PIN_CAN_*` constants |
| Adafruit Feather M4 CAN Express | Built-in MCAN controller | ATSAME51 with TJA1051T/3 transceiver, requires boost enable (`PIN_CAN_BOOSTEN`) |
| ESP32 + MCP2515 module | MCP2515 (external SPI) | Default: CS=GPIO5, INT=GPIO4. Check module crystal (8 vs 16 MHz) |
| ESP32-S3 + CAN transceiver | Built-in TWAI controller | Only needs a transceiver (e.g. SN65HVD230), no MCP2515 |
| Arduino UNO + MCP2515 module | MCP2515 (external SPI) | Default: CS=D10, INT=D2. Check module crystal (8 vs 16 MHz) |

All pin assignments are configurable at the top of each sketch. CAN bus connection to the vehicle must run at **500 kbit/s**.

## Installation

### 1. Install the Arduino IDE

Download from [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software).

### 2. Install the Board Package

Open **File → Preferences** and add the appropriate URL to **Additional Board Manager URLs**, then install the board package via **Tools → Board → Boards Manager**:

| Board | Board Manager URL | Package to install |
|---|---|---|
| Adafruit Feather RP2040 CAN | `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json` | Raspberry Pi Pico/RP2040 |
| Adafruit Feather M4 CAN Express | `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json` | Adafruit SAMD Boards |
| ESP32 / ESP32-S3 | `https://espressif.github.io/arduino-esp32/package_esp32_index.json` | esp32 by Espressif |
| Arduino UNO | *(built-in)* | Arduino AVR Boards |

### 3. Install Required Libraries

Install the required library for your board via **Sketch → Include Library → Manage Libraries…**:

| Board | Library | Notes |
|---|---|---|
| Adafruit Feather RP2040 CAN | **MCP2515** by autowp | CAN controller driver (`mcp2515.h`) |
| Adafruit Feather M4 CAN Express | **Adafruit CAN** | CANSAME5x driver (`CANSAME5x.h`) |
| ESP32 + MCP2515 | **MCP2515** by autowp | CAN controller driver (`mcp2515.h`) |
| ESP32-S3 | *(none)* | Uses built-in ESP-IDF TWAI driver |
| Arduino UNO + MCP2515 | **MCP2515** by autowp | CAN controller driver (`mcp2515.h`) |

### 4. Select Your Hardware Target

Near the top of the sketch for your board, change the `HW` define to match your vehicle:

```cpp
#define HW_TARGET TARGET_HW3  // Change to TARGET_LEGACY, TARGET_HW3, or TARGET_HW4
```

### 5. Upload

1. Connect your board via USB.
2. Select the correct board and port under **Tools**.
3. Click **Upload**.

### 6. Wiring

The recommended connection point is the [**X179 connector**](https://service.tesla.com/docs/Model3/ElectricalReference/prog-233/connector/x179/):

| Pin | Signal |
| --- | ------ |
| 13  | CAN-H  |
| 14  | CAN-L  |

Connect your board's CAN-H and CAN-L lines to pins 13 and 14 on the X179 connector.

The recommended connection point for **legacy Model 3 (2020 and earlier)** is the [**X652 connector**](https://service.tesla.com/docs/Model3/ElectricalReference/prog-187/connector/x652/) if the vehicle is not equipped with the X179 port (varies depending on production date):
| Pin | Signal |
|-----|--------|
| 1 | CAN-H |
| 2 | CAN-L |

Connect your board's CAN-H and CAN-L lines to pins 1 and 2 on the X652 connector.

**Important:** If your board or CAN module has an onboard 120 Ω termination resistor, **cut or remove it**. The vehicle's CAN bus already has its own termination, and adding a second resistor will cause communication errors. This applies to Adafruit Feather CAN boards and many MCP2515 modules.

## Speed Profiles

The speed profile controls how aggressively the vehicle drives under FSD. It is configured differently depending on the hardware variant:

### Legacy (HW3 Retrofit)

Because the Legacy variant transmits follow distance differently, it uses a **speed offset value** (in km/h) to select the profile:

| Speed Offset (km/h) | Profile |
| ------------------- | ------- |
| 28                  | Chill   |
| 29                  | Normal  |
| 30                  | Hurry   |

### HW3 & HW4 Profiles

| Distance | Profile (HW3) | Profile (HW4) |
| :------- | :------------ | :------------ |
| 2        | ⚡ Hurry      | 🔥 Max        |
| 3        | 🟢 Normal     | ⚡ Hurry      |
| 4        | ❄️ Chill      | 🟢 Normal     |
| 5        | —             | ❄️ Chill      |
| 6        | —             | 🐢 Sloth      |

## Serial Monitor

Open the Serial Monitor at **115200 baud** to see live debug output showing FSD state and the active speed profile. Disable logging by setting `enablePrint = false`.

## My Setup

I'm personally using an [Adafruit Feather M4 CAN Express with ATSAME51](https://www.antratek.nl/feather-m4-can-express-with-atsame51). It was easy to source, has an onboard MCP25625 CAN controller, and is a great fit for this application.

## Roadmap

This fork aims to expand on the original project:

- **Installation guide** — a step-by-step guide with photos covering wiring, flashing, and verifying the setup.
- **Proper connector and power** — replace the current jumper-wire setup with a proper connector cable like the [EnhAuto Tesla Gen 2 Cable](https://www.enhauto.com/products/tesla-gen-2-cable?variant=41214470094923), and connect to the same diagnostic port that the [EnhAuto S3XY Commander](https://www.enhauto.com/) uses — providing both CAN bus data and power through a single connection point. This is a similar approach to what [@mikegapinski](https://x.com/mikegapinski) uses with his Tesla Android Diagnostic Tool.
- **Enclosure** — a 3D-printable or off-the-shelf enclosure to protect the board and make the installation clean and permanent.
- **More supported boards** — e.g. Teensy 4.x (FlexCAN), STM32 + MCP2515.
- **New features** — contributions and ideas are welcome via issues and pull requests.

## Warning

**This project is for testing and educational purposes only.** Sending incorrect CAN bus messages to your vehicle can cause unexpected behavior, disable safety-critical systems, or permanently damage electronic components. The CAN bus controls everything from braking and steering to airbags — a malformed message can have serious consequences. If you don't fully understand what you're doing, **do not install this on your car**.

## Disclaimer

**Use this project at your own risk.** Modifying CAN bus messages on a vehicle can lead to unexpected or dangerous behavior. The authors accept no responsibility for any damage to your vehicle, injury, or legal consequences resulting from the use of this software. This project may void your vehicle warranty and may not comply with road safety regulations in your jurisdiction. Always keep your hands on the wheel and stay attentive while driving.

## License

This project is licensed under the **GNU General Public License v3.0** — see the [GPL-3.0 License](https://www.gnu.org/licenses/gpl-3.0.html) for details.
