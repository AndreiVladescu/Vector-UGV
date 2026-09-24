# V.E.C.T.O.R.

An 18-DOF hexapod research platform: a smart controller per leg on CAN, a Raspberry Pi CM5 carrier, camera and SDR payload, ELRS / Wi-Fi / 4G links. Built for the [PCBWay 9th Project Design Contest](https://www.pcbway.com/activity/9th-project-design-contest.html).

- `docs/`: spec, connector pinouts, power budget, bring-up notes
- `hardware/`: one KiCad project per board, shared library in `lib/`
- `mechanical/`: Fusion 360 exports (STEP, STL), harness drawings
- `firmware/`: STM32C092 code for the leg and power nodes, CAN bootloader
- `software/`: CM5 / Pi 5 side: ROS 2 workspace, tools, Docker
- `protocol/vector.dbc`: CAN message definitions, used by firmware and host

Start with `docs/spec.md`.
