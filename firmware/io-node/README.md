# IO node

The STM32C092KCT6 on the CM5 carrier (U30, sheet `io_mcu`). It takes the slow serial devices off the CM5 and talks to it over one UART: ExpressLRS (CRSF), the LD19 lidar, the MAX-M10S GNSS, the RFM95W LoRa module, battery / 5 V / NTC sensing, the buzzer and the LTE supply. It also sends a LoRa position beacon on its own, so the robot can be found with the CM5 down.

Runs on the 48 MHz HSI (no crystal, no CAN), linked at the start of flash without the CAN bootloader. Logic in `src/`, tested on the PC (`test/test_io.c`, with a simulated SX1276 in `sim/`); `stm32/` holds the drivers (register-level UARTs, SPI, ADC and timers) and the pin map.

```sh
cmake -S firmware/io-node -B build/io && cmake --build build/io && ctest --test-dir build/io
cmake -S firmware/io-node/stm32 -B build/io-stm32 && cmake --build build/io-stm32
```

## Pins

| Pin | Port | Use |
|---|---|---|
| 19, 21 | PA9, PA10 | USART1 to the CM5 (GPIO4/5, its UART2), 1 Mbaud; ROM bootloader |
| 9, 10 | PA2, PA3 | USART2 to the ELRS receiver, CRSF 420 kbaud |
| 8 | PA1 | USART4 RX from the LD19, 230400 baud |
| 32, 1 | PB8, PB9 | USART3 to the MAX-M10S, 9600 baud NMEA |
| 2 | PC14 | GNSS PPS (input, unused for now) |
| 16 | PB1 | TIM3_CH4, LD19 speed PWM at 30 kHz |
| 11 | PA4 | TIM14_CH1, buzzer |
| 26–29 | PA15, PB3–5 | RFM95W NSS, SCK, MISO, MOSI (SPI1, 6 MHz) |
| 23, 22, 20 | PA12, PA11, PC6 | RFM95W NRST, DIO0, DIO1 |
| 7, 13, 14 | PA0, PA6, PA7 | ADC: VBAT (100k/22k), 5 V (10k/10k), NTC (10k pull-up, B 3380) |
| 15, 17 | PB0, PB2 | LTE_EN (TPS62933), LTE_STATUS |
| 3 | PC15 | LED: 1 Hz with the CM5 talking, 4 Hz without |

## Link to the CM5

UART2 on the CM5 (`dtoverlay=uart2-pi5`, `/dev/ttyAMA2`), 1 Mbaud 8N1. A message is a type byte, the payload and a CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, little endian) over both, COBS-encoded and ended with 0x00. Payload up to 250 bytes; numbers little endian. The ROS side is `vector_io/link.py`.

| Type | Direction | Payload |
|---|---|---|
| 0x01 STATUS | to CM5, 10 Hz | u32 firmware, u32 uptime ms, u8 reset cause, u8 flags, u16 VBAT mV, u16 5 V mV, i16 NTC 0.1 °C (-32768 = open/short), u8 sats, u8 fix quality, u16 beacons sent, u16 messages dropped, u16 bad CRSF / LD19 / NMEA / link |
| 0x02 CRSF | to CM5 | a CRSF frame, sync to CRC |
| 0x03 LIDAR | to CM5 | an LD19 packet, 47 bytes |
| 0x04 NMEA | to CM5 | a sentence, `$` to the checksum |
| 0x05 LORA_RX | to CM5 | i16 RSSI dBm, i8 SNR × 4, the packet |
| 0x06 LORA_TX | to CM5 | u8 result of a LORA_SEND: 0 sent, 1 busy, 2 duty-cycle wait, 3 no radio |
| 0x81 CRSF_OUT | to IO | a CRSF frame for the receiver (telemetry) |
| 0x82 LIDAR_PWM | to IO | u16 duty in 0.1 %; 0 holds the pin low and the LD19 runs its own 10 Hz |
| 0x83 BEEP | to IO | u16 Hz, u16 on ms, u16 off ms, u8 count (0 stops) |
| 0x84 LTE_POWER | to IO | u8 on |
| 0x85 LORA_SEND | to IO | up to 64 bytes |
| 0x86 BEACON | to IO | u16 interval in s, 0 = off (default 30) |
| 0x87 BOOTLOADER | to IO | u32 0x746f6f62 ("boot"): reset into ST's ROM bootloader |

Status flags: bit 0 LTE_EN, 1 LTE_STATUS, 2 RC channels in the last 0.5 s, 3 LD19 packet in the last 0.5 s, 4 NMEA in the last 2 s, 5 GGA fix, 6 RFM95W found, 7 a message from the CM5 in the last 2 s.

## LoRa

869.525 MHz (g3 sub-band, 10 % duty cycle), SF9, 125 kHz, CR 4/5, CRC on, sync word 0x12, 14 dBm. After every packet the radio stays quiet for 9× its air time. The beacon is 16 bytes (165 ms on air): `'V'`, u8 sequence, u8 fix quality (0 = last known position), u8 sats, i32 lat × 1e7, i32 lon × 1e7, i16 altitude m, u16 VBAT in 10 mV. Any RFM95W or SX127x with the same settings receives it.

## Flashing

First time over the TC2050 (J30) with `stm32c0/flash_swd.sh`, or from the CM5 over SWD by closing JP30–JP32. After that from the CM5 over the link: `vector_io` sends BOOTLOADER, the MCU resets into the ROM bootloader on USART1, and `stm32flash` writes the image:

```sh
ros2 run vector_io io_flash.py build/io-stm32/io-node.bin
```

The ROM bootloader also listens on USART2, where the ELRS receiver sends CRSF; stm32flash's 0x7F on USART1 normally wins, but if it doesn't, unpower the receiver (JP33) for the flash.
