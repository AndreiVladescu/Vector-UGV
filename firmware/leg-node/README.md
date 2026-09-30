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
| 13 | PA6 | ADC1_IN6 | ADC_6V0 (15k / 10k) |
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
| 27 | PB3 | in, pull-down | LEG_ID_B0 |
| 32 | PB8 | in, pull-down | LEG_ID_B1 |
| 1 | PB9 | in, pull-down | SIDE_BIT |
| 28 | PB4 | in | TOF_INT |
| 29 | PB5 | out | TOF_XSHUT |
| 26 | PA15 | out | LED |
| 6 | PF2 | NRST | |

PA7 is the DBG test pad: high while each 1 ms tick runs (ToF, leg logic, LED), so a scope shows the loop time and any overrun; the pulse width should stay well under 1 ms. Spare: PC6.

Leg ID: jumpers to 3V3, read with the internal pull-downs. Two per cell give its position, bridged as B1B0 = 01 for cell A next to the CAN-in connector (leg 1), 10 for B, 11 for C at the outer end (leg 3). JP107 sets the side for the whole board: open = left, bridged = right. The right board is the same PCB turned 180°, so its cell A sits at the other end of the body and the order runs backwards: left A/B/C = L1/L2/L3 (nodes 1–3), right A/B/C = R3/R2/R1 (nodes 6/5/4). R1 is then the front leg on both sides, as in `legs.yaml`. This assumes cell A is at the front on the left side; if the chassis puts it at the rear, swap the two sides in `leg_node_from_straps()` (`src/leg_id.h`). Position 00 means the jumpers weren't set: the leg stays off with a config fault and blinks fast. `leg_config.py read` shows the raw jumpers.

Clocks: the C0 has no PLL, so the core runs from HSI48 (48 MHz, divider 1) and FDCAN from the 40 MHz crystal (20 time quanta per bit at 1 Mbit/s; 25 MHz also works, `-DSIDE_HSE=25000000`). HSI alone is ±1 %, too loose for CAN between six nodes. TIM1 runs at 1 MHz (prescaler 47, period 19999), so pulse widths are set in µs. The ADC must use a clock divider: CubeMX shows 48 MHz, above the C0's ADC limit.

The schematic still needs the ADC_VBAT and ADC_6V0 dividers, and the pot inputs changed to 10k series + 100 nF (see `docs/servos.md`).

## What the leg does

- **Off** until the CM5 sends `LEG_CMD` with enable and SYNC is running, and every joint is calibrated.
- **Wake**: buck on, then per joint: start the pulses at the measured position + 30 µs, make a 5° test move (away from whichever end of the range is near) and check the wiper moved by that much. A servo that ignores its first pulse gets sent past its position and tried again, three times, then the leg faults.
- **Active**: follows the commanded angles, slew-limited to 400°/s, clamped to the joint limits.
- **Crouch**: SYNC lost for 100 ms: moves slowly to a crouch, then off after 2 s. SYNC back during the crouch returns to active.
- **E-stop** (pin or SYNC flag): off at once, calibration and self-test included (they reply done, as failed).
- Joint settings, defaults and saving only while the servos are off: a flash erase stalls the CPU for ~30 ms.
- **Protection**, in any powered state, goes to fault with the buck off:
  - leg current above 5 A for about 500 ms (a leaky count, so short peaks and brief dips don't matter); three stalled MG996Rs draw ~7.5 A,
  - NTC at 85 °C; the leg can't wake again until it's below 70 °C,
  - buck power-good low for 20 ms.

  Overload and buck faults clear on the next enable. During calibration the sweep stops and reports failed.
- While the servos are unpowered their pots read 0 V, so the leg keeps reporting the last good angles.
- The wiper is the median of 16 samples per joint per 1 ms tick.
- Replies to every SYNC with `LEG_STATE`, every fourth with `LEG_STATUS`. Without SYNC it sends both every 50 ms so the CM5 can find it.

## Diagnostics and self-test

Leg-wide keys (joint 255, see `protocol/README.md`) give the firmware version (git hash, stamped at every build), why it last reset (the bootloader reads the RCC flags before its own restart and passes them on), CAN bus-off and dropped-frame counts, uptime, the current amp zero and the divider corrections.

The current amp's zero is learned continuously while the buck is off and subtracted while it's on. VBAT and 6V0 have a stored gain for divider tolerance.

Self-test is for a fresh board, servos optional: supplies, power-good, current amp, NTC, ToF and which servos are plugged in, one line each. `--vbat` with a meter reading stores the VBAT correction:

```sh
ros2 run vector_hw leg_config.py --channel can0 selftest all
ros2 run vector_hw leg_config.py --channel can0 selftest R1 --vbat 15.92 --save
ros2 run vector_hw leg_config.py --channel can0 read R1      # version, reset cause, errors, then the joints
```

The CAN controller only lets SYNC and frames for its own node into the receive FIFO, so the other legs' traffic costs nothing.

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
build/leg-node/test_leg                 # 179 checks: restart quirk, protection, config slots, self-test, e-stop, ToF
build/leg-node/test_boot                # bootloader protocol against a RAM flash
build/leg-node/sim_legs vcan0           # six legs on a virtual CAN bus, bootloader included
```

## STM32 build

`stm32/` builds the same logic for the chip: `main.c` sets up TIM1, the ADC with DMA, I2C, the two flash config pages and the watchdog, and implements `hal.h`; `board.c` has the pins and the leg ID. The rest is shared with the power board in `firmware/stm32c0/`: clocks, FDCAN (`mcu.c`), the bootloader, linker scripts, the build helper and `flash_swd.sh`. ST's HAL and CMSIS come from their GitHub repos at pinned tags (fetched by CMake). The application is about 33 KB, the bootloader 7 KB.

```sh
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi
cmake -S firmware/leg-node/stm32 -B build/leg-stm32 -DBOARD=nucleo -DLEG_ID=1   # or -DBOARD=side
cmake --build build/leg-stm32                                   # -> bootloader.bin, leg-node.bin
```

The first time, over SWD (ST-LINK, or the Nucleo's own), with `firmware/stm32c0/flash_swd.sh build/leg-stm32` (asks for each of the three cells in turn; `build/nucleo 1` for one chip). It erases, writes and verifies both images, then write-protects the bootloader pages 0–7 (option bytes `WRP1A_STRT`/`WRP1A_END`; the names are from the G0 and still to be checked on the C0). By hand:

```sh
STM32_Programmer_CLI -c port=SWD -e all -w bootloader.bin 0x08000000 -w leg-node.bin 0x08004000 -rst
```

After that, over CAN from the CM5 or a PC, legs off:

```sh
ros2 run vector_hw leg_config.py --channel can0 flash R1 build/leg-stm32/leg-node.bin
ros2 run vector_hw leg_config.py --channel can0 flash all build/leg-stm32/leg-node.bin
```

## Bootloader

Flash map: bootloader 0x08000000 (16 KB), application 0x08004000 (236 KB), config 0x0803F000 and 0x0803F800 (2 KB each, kept across updates). Each save goes to the slot with the older copy and bumps a sequence number; at boot the newest one with a good CRC wins, so losing power mid-save only loses that save. After bus-off the CAN controller is restarted, in the application and the bootloader. The bootloader reads the same jumpers.

- The application jumps to the bootloader on an `ENTER` frame, but only while the leg is off or faulted; otherwise it answers busy. It sets a flag in the top 16 bytes of RAM and resets.
- After any reset the bootloader listens for 200 ms, then resets into the application. So a leg with firmware that crashes early can still be reflashed: run `flash` and power-cycle the side board.
- It stays when asked to or when there's no valid image; the LED blinks fast.
- The first 8 bytes of the image (stack pointer and reset vector) are written last, after the CRC32 over the whole image checks out. An interrupted or corrupt upload leaves no runnable image, and the leg waits in the bootloader.
- One ack per 6-byte frame keeps the 3-deep FDCAN receive FIFO from overflowing while flash is busy. 33 KB takes a few seconds per leg.

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

LED: on = active; while off, the leg number every 3 s (short blinks = left, long = right, so R2 is two long blinks); fast blink = waking, fault or unset leg ID; double blink = calibrating or self-test; 10 Hz = waiting in the bootloader.

## ToF

`src/vl53l1x.c` is a small C port of ST's VL53L1X ultra lite driver, the one inside the SparkFun Arduino library; its init sequence, default config and timing tables are checked register by register against SparkFun's copy and Pololu's library, and `test_leg` runs it against a register model (`sim/tof_sim.c`).

I2C1 at 400 kHz on PB6/PB7, XSHUT on PB5 (TOF_INT on PB4 is unused, the driver polls every 5 ms). Short mode, 20 ms budget, a reading every 25 ms. The driver is a state machine that never blocks for more than one I2C transfer (the default config goes over in 23-byte pieces): it looks for the sensor every 500 ms, brings it up, and starts over after three bus errors in a row or a second without a range, so a sensor plugged in, unplugged or browned out comes back on its own. While it waits it holds XSHUT low, which also frees a bus the sensor was holding (a glitch on the cable through the coxa), and the I2C block is reset with it. A stuck bus costs one 25 ms HAL timeout per failed transfer before that. `LEG_STATUS` carries 0 with no sensor or no reading in the last 200 ms, 65535 when nothing valid is in range.

On the Nucleo the breakout goes to the same pins on the morpho header. To check a breakout on its own first, the Pololu `VL53L1X` Arduino library's Continuous example on the Uno does the same thing.

## Nucleo bring-up

In this order; each step only needs the ones before it. `lc` is `ros2 run vector_hw leg_config.py --channel can0`.

Wiring: Nucleo CAN header to the CANable, 120 Ω at both ends. Servos from the bench supply at 6 V, grounds joined: signals to PA8/PA9/PA10, wipers to PA0/PA1/PA4. CANable up with `sudo ip link set can0 up type can bitrate 1000000`.

1. **Build and flash.** `cmake -S firmware/leg-node/stm32 -B build/nucleo -DBOARD=nucleo -DLEG_ID=1 && cmake --build build/nucleo`, then `firmware/stm32c0/flash_swd.sh build/nucleo 1`. If the option-byte step fails, flash by hand and fix the names in the script. `STM32_Programmer_CLI -c port=SWD -r32 0x08004000 8` should show `200077F0` and a reset vector in 0x0800xxxx.
2. **Application starts.** LD1 blinks slowly (leg off). A fast 10 Hz blink means it stayed in the bootloader: the image is missing or the jump failed.
3. **CAN.** `candump can0` shows `021` and `031` every 50 ms. Nothing: check the termination and the bit rate, and `ip -s -d link show can0` for errors. `lc status` prints L1 off, uncalibrated; `lc read L1` shows the git hash of the build and the last reset: power after a power cycle, software after `flash`.
4. **Bootloader over CAN.** `lc flash L1 build/nucleo/leg-node.bin` ends with `L1: running`. This proves the RAM flag survives the reset and the jump back.
5. **Recovery.** Erase the first application page, `STM32_Programmer_CLI -c port=SWD -e 8 -rst`: fast blink, and `candump` shows one `081` frame. `lc flash` brings it back.
6. **PWM and ADC.** `lc calibrate L1 --joint coxa` sweeps the servo on PA8. mid and slope should land within a few mV of that servo's row in `docs/servos.md` (different ADC and reference than the Uno). Then femur and tibia.
7. **Config slots.** Calibrate all three with `--save`, press reset, and `lc read L1` still shows the values. After a second save, `-r32 0x0803F000 1` and `-r32 0x0803F800 1` both read `56454732` (the config magic).
8. **Walking.** SYNC and LEG_CMD from `robot.launch.py hardware:=can can_interface:=can0`: L1 wakes (the LED goes steady) and follows. Pull the CAN cable: it crouches, then turns off after 2 s.
9. **Bus-off.** With the leg off, short CANH to CANL for a second while `candump` runs. The frames come back within milliseconds of letting go, without a reset.
10. **ToF.** Plug a breakout into PB6/PB7 with XSHUT on PB5 while it runs. Within a second, bytes 0–1 of `031` follow a hand in front of it. Unplug and replug: it comes back.

The Nucleo can't test current, temperature, supply sense, buck power-good or e-stop: they aren't wired and read fixed values. Those wait for the side board.
