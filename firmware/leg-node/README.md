# Leg node firmware

One STM32C092KCT6 (LQFP32) per leg cell. The leg logic is plain C in `src/` with the hardware behind `hal.h`, so the same code runs on the chip, in the unit tests and in `sim_legs` (six legs with simulated servos on SocketCAN).

## Pins

From `stm32/leg-node.ioc`, checked by loading it in CubeMX 6.18. Labels match the side-board schematic.

| Pin | Port | Function | Label |
|---|---|---|---|
| 2, 3 | PC14, PC15 | HSE crystal, 40 MHz (same part as the MCP251863 on the carrier) | |
| 7 | PA0 | ADC1_IN0 | ADC_POT_COXA |
| 8 | PA1 | ADC1_IN1 | ADC_POT_FEMUR |
| 9 | PA2 | ADC1_IN2 | ADC_POT_TIBIA |
| 10 | PA3 | ADC1_IN3 | ADC_I_LEG (INA181 out) |
| 11 | PA4 | ADC1_IN4 | NTC_SENSE |
| 12 | PA5 | ADC1_IN5 | ADC_VBAT (100k / 15k) |
| 13 | PA6 | ADC1_IN6 | ADC_6V0 (10k / 6.8k) |
| 18 | PA8 | TIM1_CH1 | TIM_PWM_COXA |
| 19 | PA9 | TIM1_CH2 | TIM_PWM_FEMUR |
| 21 | PA10 | TIM1_CH3 | TIM_PWM_TIBIA |
| 22 | PA11 | FDCAN1_RX | |
| 23 | PA12 | FDCAN1_TX | |
| 24 | PA13 | SWDIO | |
| 25 | PA14 | SWCLK | |
| 30 | PB6 | I2C1_SCL | TOF_SCL |
| 31 | PB7 | I2C1_SDA | TOF_SDA |
| 15 | PB0 | out | BUCK_EN |
| 16 | PB1 | in | BUCK_PG |
| 17 | PB2 | in | GPIO_ESTOP (low = stop) |
| 27 | PB3 | in | LEG_ID_B0 |
| 32 | PB8 | in | LEG_ID_B1 |
| 1 | PB9 | in | LEG_ID_B2 |
| 28 | PB4 | in | TOF_INT |
| 29 | PB5 | out | TOF_XSHUT |
| 26 | PA15 | out | LED |
| 6 | PF2 | NRST | |

Spare: PA7 (ADC1_IN7) and PC6. The leg ID jumpers give the CAN node 1–6 (L1, L2, L3, R1, R2, R3).

Clocks: the C0 has no PLL, so the core runs from HSI48 (48 MHz, divider 1) and FDCAN from the 40 MHz crystal (20 time quanta per bit at 1 Mbit/s; 25 MHz also works, `-DSIDE_HSE=25000000`). HSI alone is ±1 %, too loose for CAN between six nodes. TIM1 runs at 1 MHz (prescaler 47, period 19999), so pulse widths are set in µs. The ADC must use a clock divider: CubeMX shows 48 MHz, above the C0's ADC limit.

The schematic still needs the ADC_VBAT and ADC_6V0 dividers, and the pot inputs changed to 10k series + 100 nF (see `docs/servos.md`).

## What the leg does

- **Off** until the CM5 sends `LEG_CMD` with enable and SYNC is running, and every joint is calibrated.
- **Wake**: buck on, then per joint: start the pulses at the measured position + 30 µs, make a 5° test move (away from whichever end of the range is near) and check the wiper moved by that much. A servo that ignores its first pulse gets sent past its position and tried again, three times, then the leg faults.
- **Active**: follows the commanded angles, slew-limited to 400°/s, clamped to the joint limits.
- **Crouch**: SYNC lost for 100 ms: moves slowly to a crouch, then off after 2 s. SYNC back during the crouch returns to active.
- **E-stop** (pin or SYNC flag): off at once.
- While the servos are unpowered their pots read 0 V, so the leg keeps reporting the last good angles.
- The wiper is the median of 16 samples per joint per 1 ms tick.
- Replies to every SYNC with `LEG_STATE`, every fourth with `LEG_STATUS`. Without SYNC it sends both every 50 ms so the CM5 can find it.

## Calibration mode

`LEG_CONFIG` with op `CALIBRATE` sweeps a joint (or all three) down and up in 11 steps each way, 400 ms per step, fits `mV = mid + slope × (µs − 1500)` and replies with the result. Two ranges:

- **full**: 500–2500 µs, the whole servo, for a servo on the bench or a free-hanging leg.
- **limits**: only the joint's own range, `min_deg`..`max_deg` from its centre, direction and µs/deg, plus 5°. For a servo already in the leg, where the full range would hit the frame.

From the CM5 (or a PC with a CAN adapter):

```sh
ros2 run vector_hw leg_config.py --channel can0 calibrate R1 --mode limits --save
ros2 run vector_hw leg_config.py --channel can0 push      # docs/servos.md + legs.yaml -> all legs
ros2 run vector_hw leg_config.py --channel can0 status
```

A result is rejected if the slope is outside 0.5–3 mV/µs, the fit is off by more than 60 mV, or the wiper spans less than 200 mV (servo not moving).

## Build and test on the PC

```sh
cmake -S firmware/leg-node -B build/leg-node && cmake --build build/leg-node
build/leg-node/test_leg                 # 89 checks, simulated MG996Rs incl. the restart quirk
build/leg-node/sim_legs vcan0           # six legs on a virtual CAN bus
```

## STM32 build

`stm32/` builds the same logic for the chip: `main.c` sets up clocks, TIM1, the ADC with DMA, FDCAN, the flash config page and the watchdog, and implements `hal.h`. ST's HAL and CMSIS come from their GitHub repos at pinned tags (fetched by CMake). About 29 KB flash, 4 KB RAM.

```sh
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi
cmake -S firmware/leg-node/stm32 -B build/leg-stm32 -DBOARD=nucleo -DLEG_ID=1   # or -DBOARD=side
cmake --build build/leg-stm32                                                   # -> leg-node.bin
```

Flash: copy `leg-node.bin` onto the Nucleo's USB drive, or with an ST-LINK `STM32_Programmer_CLI -c port=SWD -w leg-node.bin 0x08000000 -rst`.

| | side board | NUCLEO-C092RC |
|---|---|---|
| crystal | 40 MHz on PC14/PC15 | 48 MHz on the board |
| CAN | PA11/PA12 → TCAN332 | PD0/PD1 → MCP2562FD, standby PD2 held low |
| pots | PA0, PA1, PA2 | PA0, PA1, PA4 (PA2/PA3 are the ST-LINK serial port) |
| PWM | PA8, PA9, PA10 | same |
| LED | PA15 | PA5 (LD1) |
| leg ID | jumpers on PB3/PB8/PB9 | `-DLEG_ID=n` at build time |
| buck, e-stop, current, NTC, supply sense | wired | not there; buck always "good", readings fixed |

CAN timing comes from the crystal: the smallest prescaler with at most 25 time quanta per bit, sample point near 87.5 % (25 MHz → 25 tq, 40 MHz → 20 tq, 48 MHz → 24 tq).

LED: on = active, slow blink = off, fast blink = waking or fault, double blink = calibrating.

First test on the Nucleo: servo signal to PA8, wiper to PA0, servo power from the bench supply with common ground, Nucleo CAN header to the CANable (120 Ω at both ends). Then from the PC:

```sh
ros2 run vector_hw leg_config.py --channel can0 status
ros2 run vector_hw leg_config.py --channel can0 calibrate L1 --joint coxa
```

The ToF driver (VL53L1X) isn't in yet; `LEG_STATUS` reports 0 mm.
