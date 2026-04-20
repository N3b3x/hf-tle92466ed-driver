---
layout: default
title: "🔌 Hardware Setup"
description: "Hardware wiring and connection guide for TLE92466ED driver"
nav_order: 3
parent: "📚 Documentation"
permalink: /docs/hardware_setup/
---

# Hardware Setup

This guide covers the physical connections and hardware requirements for the TLE92466ED
Six-Channel Low-Side Solenoid Driver IC.

![TLE92466ED six-channel low-side topology with parallel-pair pairing](assets/tle92466ed-topology.svg)

## Pin Connections

### Basic SPI Connections

```text
MCU              TLE92466ED
─────────────────────────
3.3V/5V   ────── VIO
GND       ────── GND
5.5-41V   ────── VBAT
SCK       ────── SCK
MOSI      ────── SI
MISO      ────── SO
CS        ────── CSN
```cpp

### Control Pins

```text
MCU              TLE92466ED
─────────────────────────
GPIO      ────── RESN (optional, active low reset)
GPIO      ────── EN (optional, active high enable)
GPIO      ────── FAULTN (optional, active low fault output)
```cpp

### Pin Descriptions

| Pin | Name | Description | Required |
|-----|------|-------------|----------|
| VBAT | Power | Battery voltage supply (5.5V - 41V) | Yes |
| VIO | Power | SPI I/O voltage (3.0V - 5.5V, typically 3.3V or 5.0V) | Yes |
| VDD | Power | Internal 5V supply output (max 10mA load) | No |
| GND | Ground | Ground reference | Yes |
| SCK | Clock | SPI clock line (up to 10 MHz) | Yes |
| SI | Data In | SPI data input (MOSI) | Yes |
| SO | Data Out | SPI data output (MISO) | Yes |
| CSN | Chip Select | SPI chip select (active low) | Yes |
| RESN | Reset | Active-low reset (optional) | No |
| EN | Enable | Active-high global enable (optional) | No |
| FAULTN | Fault | Active-low fault indication (open drain, optional) | No |
| OUT0-5 | Outputs | Low-side switch outputs (6 channels) | Yes |

## Power Requirements

- **VBAT (Battery Supply)**: 5.5V - 41V (12V or 24V typical for automotive)
- **VIO (I/O Supply)**: 3.0V - 5.5V (3.3V or 5.0V typical)
- **VDD (Internal 5V)**: Output from IC, max 10mA load
- **Current per Channel**: Up to 2A single channel, 4A parallel mode
- **Power Supply**: Clean supply with decoupling capacitors (100nF ceramic + 10µF tantalum recommended)

## SPI Configuration

- **Mode**: SPI Mode 1 (CPOL=0, CPHA=1)
- **Speed**: Up to 10 MHz
- **Bit Order**: MSB first
- **CS Polarity**: Active low (CSN)
- **Data Format**: 32-bit frames with 8-bit CRC (SAE J1850)
- **Frame Structure**: CRC[31:24] + Address[23:17] + R/W[16] + Data[15:0]

## Load Connections

### Single Channel Mode

Connect the load between OUTx and VBAT (or positive supply):

```cpp
VBAT ────[Load]──── OUTx ──── GND
```cpp

### Parallel Channel Mode

For higher current, connect channels in parallel (pairs: 0/3, 1/2, 4/5):

```cpp
VBAT ────[Load]──── OUT0 ──── GND
                  └──── OUT3 ──── GND
```cpp

## Physical Layout Recommendations

- Keep SPI traces short (< 10cm recommended)
- Use ground plane for noise reduction
- Place decoupling capacitors (100nF ceramic + 10µF tantalum) close to VBAT and VIO pins
- Route clock and data lines away from noise sources
- Keep high-current traces (OUT0-5) away from sensitive signals
- Use appropriate trace widths for high-current paths (1A = ~20 mils minimum)
- Add flyback diodes for inductive loads (solenoids, valves)

## Example Wiring Diagram

```text
                    TLE92466ED
                    ┌─────────┐
       12V/24V ─────┤ VBAT    │
       3.3V    ─────┤ VIO     │
       GND     ─────┤ GND     │
       SCK     ─────┤ SCK     │
       MOSI    ─────┤ SI      │
       MISO    ─────┤ SO      │
       CS      ─────┤ CSN     │
                    └─────────┘
                      │
                      │ OUT0
                      ├───[Solenoid]─── VBAT
                      │
                      │ OUT1
                      ├───[Valve]─── VBAT
```cpp

## Protection Components

### Flyback Diodes

For inductive loads (solenoids, valves), the TLE92466ED has internal protection, but external flyback diodes may be needed for very high inductance loads:

```cpp
VBAT ────[Flyback Diode]─── OUTx
         (cathode to VBAT)
```cpp

### Current Limiting

The TLE92466ED has built-in overcurrent protection (OCP) and current regulation via ICC, but you may want to add external fuses for additional safety.

## Next Steps

- Verify connections with a multimeter
- Proceed to [Quick Start](quickstart.md) to test the connection
- Review [Platform Integration](platform_integration.md) for software setup

---

**Navigation**
⬅️ [Quick Start](quickstart.md) | [Next: Platform Integration ➡️](platform_integration.md) | [Back to Index](index.md)
