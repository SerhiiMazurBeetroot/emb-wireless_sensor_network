# Hardware

Wiring and power details for the sensor nodes and the gateway. For the
project overview, see the [main README](../README.md).

## Contents

- [Hardware](#hardware)
  - [Contents](#contents)
  - [Power supply](#power-supply)
  - [Battery measurement](#battery-measurement)
  - [Current measurement](#current-measurement)
  - [Pin assignments](#pin-assignments)
    - [nRF24L01+ header](#nrf24l01-header)
    - [Gateway — ESP32-S3](#gateway--esp32-s3)
    - [Sensor nodes — STM32F401](#sensor-nodes--stm32f401)
  - [Build notes](#build-notes)

> [!WARNING]
> The nRF24L01+ modules must be powered from **3.3 V**. Do not connect the
> nRF24L01+ VCC pin to 5 V. All GPIO signals use 3.3 V logic.

## Power supply

Each sensor node is powered by a single 18650 lithium-ion cell:

```text
18650 cell → TP4056 charging/protection module → HT7333 → 3.3 V rail
```

The TP4056 is used for battery charging. Battery protection is available
only when using a module variant that includes a protection circuit. The
HT7333 provides the regulated 3.3 V supply.

A 47 µF capacitor is installed close to the power pins of each nRF24L01+
radio module.

## Battery measurement

Battery voltage is measured through a 100 kΩ / 100 kΩ divider connected to
PA1 (`ADC1_IN1`). The firmware scales the ADC reading as follows:

```c
battery_mv = raw * 6600 / 4095;
```

A measured divider midpoint of 1.860 V corresponds to an estimated battery
voltage of approximately 3.72 V.

Notes:

- The `LOW_BAT` flag is raised below 3.4 V.
- The ADC is referenced to the 3.3 V rail, so the reading becomes less
  accurate when the battery voltage approaches the dropout of the regulator.
- The divider is always connected. A 1 MΩ / 1 MΩ divider with a 100 nF
  capacitor at the ADC input used about 2 µA.

## Current measurement

An INA219 module can be placed in series with the node supply (for example
in a jumper gap) to measure the awake-state current and bus voltage.

- With a typical 0.1 Ω shunt module its resolution is on the order of
  100 µA, so it cannot resolve STOP-mode current.
- The INA219 itself draws about 1 mA, which is added to the measured value
  when it is powered from the node 3.3 V rail.
- STOP-mode current should be measured with a multimeter, with the INA219
  module removed and UART disconnected.

## Pin assignments

### nRF24L01+ header

| Pin | Signal      |
| --- | ----------- |
| 1   | GND         |
| 2   | VCC (3.3 V) |
| 3   | CE          |
| 4   | CSN         |
| 5   | SCK         |
| 6   | MOSI        |
| 7   | MISO        |
| 8   | IRQ         |

### Gateway — ESP32-S3

| Signal                        | GPIO   |
| ----------------------------- | ------ |
| nRF24 SCK                     | GPIO12 |
| nRF24 MOSI                    | GPIO11 |
| nRF24 MISO                    | GPIO13 |
| nRF24 CSN                     | GPIO10 |
| nRF24 CE                      | GPIO14 |
| nRF24 IRQ                     | GPIO7  |
| OLED SDA (I²C address `0x3C`) | GPIO8  |
| OLED SCL                      | GPIO9  |

### Sensor nodes — STM32F401

| Signal                                  | Pin           | Notes                             |
| --------------------------------------- | ------------- | --------------------------------- |
| nRF24 SCK (SPI2)                        | PB10          |                                   |
| nRF24 MISO (SPI2)                       | PB14          |                                   |
| nRF24 MOSI (SPI2)                       | PB15          |                                   |
| nRF24 CSN                               | PB12          |                                   |
| nRF24 CE                                | PA8           |                                   |
| nRF24 IRQ                               | not connected | ACK waiting uses polling          |
| BME280 SCK (SPI1)                       | PA5           | Node 1 only                       |
| BME280 MISO (SPI1)                      | PA6           | Node 1 only                       |
| BME280 MOSI (SPI1)                      | PA7           | Node 1 only                       |
| BME280 CS                               | PA4           | Node 1 only                       |
| AHT20 SCL (I²C1, address `0x38`)        | PB6           | Node 2 only                       |
| AHT20 SDA (I²C1)                        | PB7           | Node 2 only                       |
| INA219 SCL / SDA (I²C1, address `0x40`) | PB6 / PB7     | Where fitted; shares the I²C1 bus |
| Battery voltage (`ADC1_IN1`)            | PA1           |                                   |
| Debug TX (USART1)                       | PA9           |                                   |
| Debug RX (USART1)                       | PA10          |                                   |

## Build notes

- Keep the SPI wires to the nRF24L01+ short, and place the 47 µF capacitor
  (optionally with a 100 nF ceramic capacitor) directly at the radio's VCC
  and GND pins. Missing decoupling is a common cause of unreliable links.
- Keep the antenna end of the nRF24L01+ module clear of metal and ground
  copper.
- If a Black Pill board is used, remove its power LED when measuring sleep
  current.
- Disconnect the ST-Link 3.3 V line when the node runs from the battery, and
  disconnect the debugger entirely for sleep-current measurements.
