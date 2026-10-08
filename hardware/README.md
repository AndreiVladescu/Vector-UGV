# Hardware

One KiCad project per board: `side-board/`, `power-board/`, `carrier-cm5/`. The lib tables in each point to the shared library in `lib/`, so custom symbols and footprints go there.

- Connector pinouts come from `docs/interfaces.md`. Use the same footprint and pin order on both ends of a cable.
- Side board: draw the leg cell as one hierarchical sheet, use it three times, and copy the layout with KiCad's multichannel tools.
- Tag each fabbed revision (`side-board-revA`) and put the revision and git hash on the silkscreen. Gerbers, BOM and pick-and-place go in `<board>/production/<rev>/`.
- Hand assembly: 0402 minimum, one side where possible, test points and 0 Ω links on every rail.


`BOM_LCSC.csv` is a second way to buy the main parts for one robot: LCSC part numbers, quantities, prices and stock checked on 2026-10-08, and where each part is cheaper (LCSC, Mouser or TME). It loads straight into LCSC's BOM tool. Passives aren't in it yet.

## Power rails

Currents and loads are in `docs/power-budget.md`. No 5 V on the side boards: the CAN transceivers are 3.3 V parts (TI TCAN332DR).

```
PACK 4S3P 18650 (12.0–16.8 V)
│
└─ POWER BOARD ── BQ76952 high-side FETs ── VBAT
   │                                  (charging: BQ25798, fed by GX-12 12–24 V or USB-C PD 20 V)
   │
   ├─ LM5069 (hot-swap + e-stop) ── VBAT_L ──► SIDE BOARD L
   │                                  ├─ TPS56A37 ── 6V0_LEG1 ── 3× MG996R (+ pots)
   │                                  ├─ TPS56A37 ── 6V0_LEG2 ── 3× MG996R (+ pots)
   │                                  ├─ TPS56A37 ── 6V0_LEG3 ── 3× MG996R (+ pots)
   │                                  └─ low-Iq buck ── 3V3_LEG ── 3× STM32C092, 3× CAN xcvr,
   │                                                              3× VL53L1X, 3× INA181
   │
   ├─ LM5069 (hot-swap + e-stop) ── VBAT_R ──► SIDE BOARD R   (same as L)
   │
   ├─ 5 V / 6 A buck ── 5V_SYS ──► CARRIER
   │                                  ├─ CM5 ── 3V3 / 1V8 (CM5's own outputs)
   │                                  ├─ 2× USB-A (current-limit switches)
   │                                  ├─ SDR (AD9363), fan
   │                                  └─ 3V3_M2 ── Hailo M.2 (optional)
   │
   ├─ VBAT_SYS ──► CARRIER
   │                ├─ 3.8 V buck ── 3V8_LTE ── EC25-EUX
   │                └─ small 5 V buck ── gimbal (2× SG90)
   │
   └─ 3V3_PWR (TBD) ── STM32C092 on the power board, BQ76952 comms
```

3.3 V is made locally on each side board, not sent over the CAN harness: the STM32C0 ADC uses VDD as its reference, so a remote 3.3 V would put servo current noise on the pot and current readings, and a short on one side would take down the other.
