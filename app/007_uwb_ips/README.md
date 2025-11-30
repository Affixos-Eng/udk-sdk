# UWB Indoor Positioning System (007_uwb_ips)

**Version**: 4.0.0  
**Location**: `leaps-udk-examples/app/007_uwb_ips/`  
**SDK**: LEAPS UDK-SDK (Zephyr v3.1.0)  
**Hardware**: LEAPS LC13/LC14 with Murata Type 2AB module (nRF52840 + QM33120W)

## Overview

This is the UWB Indoor Positioning System application integrated into the LEAPS UDK-SDK. It implements:

- **DS-TWR** (Double-Sided Two-Way Ranging) for distance measurement
- **Trilateration** for 3D position calculation
- **Mesh routing** for position data forwarding to gateway

### Supported Boards

| Board | Module | Description |
|-------|--------|-------------|
| `leaps_lc13` | Murata Type 2AB | LC13_C1 development board |
| `leaps_lc14` | Murata Type 2AB | LC14 development board |

## Build Commands

From the `zephyrproject/` directory:

```bash
# Setup environment
source ../.venv/bin/activate
export ZEPHYR_BASE=$PWD/zephyr
source $ZEPHYR_BASE/zephyr-env.sh

# Build Tag (use leaps_lc13 for LC13_C1 boards)
west build -b leaps_lc13 leaps-udk-examples/app -d build_tag -- \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCONFIG_UWB_IPS_TAG=y \
    -DCONFIG_UWB_IPS_DEVICE_ID=1

# Build Anchor
west build -b leaps_lc13 leaps-udk-examples/app -d build_anchor -- \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCONFIG_UWB_IPS_ANCHOR=y \
    -DCONFIG_UWB_IPS_DEVICE_ID=1001

# Build Gateway Anchor
west build -b leaps_lc13 leaps-udk-examples/app -d build_gateway -- \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCONFIG_UWB_IPS_ANCHOR=y \
    -DCONFIG_UWB_IPS_GATEWAY=y \
    -DCONFIG_UWB_IPS_DEVICE_ID=1001
```

**Note**: Replace `leaps_lc13` with `leaps_lc14` for LC14 boards.

## Configuration Options

Options are set via `-DCONFIG_*` flags during build:

| Option | Description | Values |
|--------|-------------|--------|
| `UWB_IPS_TAG` | Build as Tag | y/n |
| `UWB_IPS_ANCHOR` | Build as Anchor | y/n |
| `UWB_IPS_GATEWAY` | Enable UART gateway | y/n |
| `UWB_IPS_DEVICE_ID` | Device ID | 1-65534 |
| `UWB_IPS_UPDATE_RATE_HZ` | Tag update rate | 1-10 |
| `UWB_IPS_MAX_ANCHORS` | Max ranging anchors | 3-8 |
| `UWB_IPS_BEACON_INTERVAL_MS` | Beacon period | 1000-30000 |
| `UWB_IPS_CHANNEL` | UWB channel | 5 or 9 |

## File Structure

```
007_uwb_ips/
├── CMakeLists.txt          # Build configuration
├── README.md               # This file
├── common/
│   ├── uwb_ips.h           # Main header with types & constants
│   ├── uwb_config.c        # UWB transceiver initialization
│   ├── position_calc.c     # Least-squares trilateration
│   ├── mesh_routing.c      # Routing table management
│   ├── battery_mgmt.c      # Battery voltage monitoring
│   └── uwb_utils.c         # CRC-16 calculation
├── tag/
│   ├── tag_main.c          # Tag application entry point
│   ├── tag_twr.h           # Tag TWR interface
│   └── tag_twr.c           # DS-TWR initiator implementation
└── anchor/
    ├── anchor_main.c       # Anchor application entry point
    ├── anchor_twr.h        # Anchor TWR interface
    ├── anchor_twr.c        # DS-TWR responder implementation
    └── anchor_gateway.c    # UART gateway interface
```

## SDK Dependencies

This application uses:

### From SDK HAL (`../hal/`)
- `hal-spi.c/h` - SPI interface for DW3000
- `hal-gpio.c/h` - GPIO control
- `hal-console.c/h` - Console output

### From SDK Shared Data (`../006_ex_uwb/shared_data/`)
- `shared_functions.c/h` - TWR helper functions
- `probe_interface.c` - Device probing
- `config_options.c/h` - TX power configuration

### From SDK Driver (`../../drivers/dwt_uwb_driver/`)
- `deca_device_api.h` - DW3000 driver API

## TWR Protocol

The application implements DS-TWR (Double-Sided Two-Way Ranging):

```
TAG (Initiator)                    ANCHOR (Responder)
     │                                  │
     │────── Poll ─────────────────────>│ Record T2
     │  T1                              │
     │<───── Response ──────────────────│ T3
     │  T4                              │
     │────── Final ────────────────────>│ T6
     │  T5                              │
     │                                  │
     │     Distance = f(T1..T6) × c     │
```

### Message Types

| Code | Type | Description |
|------|------|-------------|
| 0x21 | Poll | Initiates ranging |
| 0x10 | Response | Responder acknowledgment |
| 0x23 | Final | Contains all timestamps |
| 0x30 | Position | Tag position report |
| 0x31 | Beacon | Anchor presence broadcast |

## Position Calculation

Tags calculate position using **least-squares trilateration**:

1. Range to ≥3 anchors using DS-TWR
2. Get anchor positions from beacons
3. Solve over-determined system using pseudo-inverse
4. Calculate quality metric from residual error

## Mesh Routing

Anchors form a mesh network to relay position data:

1. Gateway anchor broadcasts beacons with `hop_count=0`
2. Other anchors learn routes and increment hop count
3. Position messages route toward gateway via shortest path
4. Gateway forwards to host via UART

## Memory Usage

| Firmware | Flash | RAM |
|----------|-------|-----|
| Tag | ~99 KB | ~11 KB |
| Anchor | ~98 KB | ~10 KB |

## Hardware: Murata Type 2AB Module

The LEAPS LC13/LC14 boards use the Murata Type 2AB UWB+BLE combo module:

| Component | Part |
|-----------|------|
| UWB IC | Qorvo QM33120W |
| MCU | Nordic nRF52840 |
| Bluetooth | 5.2 LE |
| Module P/N | LBUA5QJ2AB-828 |

### Pin Connections (from SDK board DTS)

| Function | GPIO | Description |
|----------|------|-------------|
| UWB Reset | P0.15 | DW3000 reset |
| UWB IRQ | P0.25 | DW3000 interrupt |
| UWB CS | P0.20 | SPI chip select |
| SPI | SPI3 | 32 MHz |

### Datasheet

[Murata Type 2AB Datasheet](https://www.murata.com/-/media/webrenewal/products/connectivitymodule/ultra-wide-band/qorvo/type2ab/uwb-ble-module-preliminary-datasheet-sp-2ab-828i.ashx)

## Related Documentation

- [TWRDesign.md](../../../../firmware/docs/TWRDesign.md) - Protocol details
- [MeshDesign.md](../../../../firmware/docs/MeshDesign.md) - Mesh routing
- [Design.MD](../../../../Design.MD) - System architecture

