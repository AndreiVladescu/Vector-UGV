Project V.E.C.T.O.R. (Versatile Edge Compute & Tactical Observation Robot) is a highly modular, 18-axis hexapod platform engineered for autonomous traversal and low-power, localized electronic surveillance. Designed specifically for the PCBWay hardware contest, the system abandons the traditional, centralized "spaghetti-wiring" of hobbyist hexapods in favor of a commercial-grade distributed architecture. It features a centralized compute carrier board routing high-speed logic to six independent, smart-actuation leg nodes.
The Mission Profile: "Sentinel Stance"

V.E.C.T.O.R. is not designed for continuous, highly inefficient walking. It operates as a deployable Edge AI and Signals Intelligence (SIGINT) node.

    Deployment: The robot navigates rough, uneven terrain using its ToF sensors to clear obstacles and its 18 MG996R servos to execute an adaptive tripod or ripple gait.

    The Zero-Power Drop: Upon reaching its target waypoint, the hexapod splays its legs and drops its chassis flat onto the ground ("Sentinel Stance").

    Hardware Isolation: Using localized high-side P-Channel MOSFET switches on each leg board, the Raspberry Pi CM5 completely severs power to the drivetrain. The 18 servos drop to 0W consumption, instantly eliminating the robot's thermal and acoustic signatures.

    Observation & Intelligence: With the massive battery freed from holding the robot up, GPIO load switches awaken the USB 3.0 payloads. The Hailo-8 AI accelerator and an integrated SDR (Software Defined Radio) begin capturing and analyzing RF spectrum data and visual feeds, utilizing a pan/tilt CSI camera to monitor the environment for hours or days.

Master Component & Subsystem Checklist

1. The Compute & Intelligence Core
The "brain" of the hexapod, handling inverse kinematics, I2C routing, and payload management.

    Compute Module: Raspberry Pi CM5 (requires 2x Hirose DF40C-100-pin connectors).

    AI Accelerator: Hailo-8 or Google Coral TPU (mounted via M.2 Key-M 2230/2242 socket).

    Vision: Raspberry Pi CSI-2 Camera Module (mounted on an independent 2-axis SG90 gimbal).

    SDR Payload: RTL-SDR or HackRF, connected via dual stacked USB 3.0 Type-A/Type-C ports with TUSB321 orientation logic and HD3SS3212 SuperSpeed MUXes.

    Telemetry Link: ExpressLRS (ELRS) Nano Receiver wired directly to a CM5 hardware UART.

2. Main Carrier Board: Central Power & Distribution
This board takes raw battery voltage, protects the system, and generates the centralized logic rails.

    Battery Pack: Custom 4S3P or 6S2P Li-Ion pack using 21700 cells (e.g., Molicel P42A) for maximum energy density. Connected via XT60.

    Input Protection: LM74700-Q1 Ideal Diode Controller + CSD18534Q5A N-FET (zero-loss reverse polarity protection) and SMBJ30A TVS diode (voltage spike clamping).

    Main Logic Buck (5.0V @ 3A-5A): LM5145 or TPS54531 synchronous buck regulator (powers CM5, USB payloads).

    Sensors & M.2 Bucks (3.3V): TPS62088 (dedicated 3A for the AI M.2 slot) and TLV75533P (1A LDO for logic/I2C).

    Payload Load Switching: TPS22918 high-side load switches to physically cut 5V power to the SDR and USB ports during the walking phase.

3. The I2C Spine & PWM Generation
To prevent I2C address collisions and offload PWM timing from the Pi's CPU.

    I2C Multiplexer: TCA9548A (8-channel switch) separating the left legs, right legs, and ToF sensors onto physically isolated data buses.

    PWM Controllers: 2x PCA9685 16-channel chips generating 18 clean PWM signals for the legs and 2 for the camera gimbal.

    Collision Sensors: 3x TOF200C / VL53L0X Time-of-Flight sensors (Front, Mid, Rear) isolated on their own MUX channels to prevent the 0x29 address conflict.

4. Distributed Leg Boards (Quantity: 6)
Mounted directly on the limbs, these boards minimize heavy power cabling and provide granular joint data.

    Local High-Power Buck (V_batt to 6.0V): TPS563200 (or equivalent 3A-5A synchronous buck) to drop the high-voltage battery rail strictly for the three servos on that specific leg.

    Motors: 18x MG996R standard servos (3 per leg: Coxa, Femur, Tibia).

    High-Side Drivetrain Cutoff: Discrete P-Channel MOSFET (e.g., DMP3028LSD) driven by an N-Channel gate driver (2N7002), controlled by a spare PCA9685 PWM channel acting as an ON/OFF enable pin.

    Proprioception (Touch/Torque Sensing): 1x INA3221 (3-channel I2C current monitor) per leg. Measures the current draw of all three servos independently across three 100mΩ shunt resistors, allowing the AI to detect when a specific foot hits the ground or gets snagged.

Cabling & Interconnects

The modular architecture drastically cleans up the physical build. Instead of routing 54 wires (18 servos × 3 wires) to a central board, only two cables run to each leg:

    The High-Voltage Bus: A 2-wire heavy-gauge silicone cable (V_batt and GND) daisy-chained from the main board.

    The Logic Harness: A lightweight 5-wire ribbon cable (JST-GH connectors) carrying SDA, SCL, and the 3 PWM signals.

This hardware design provides a highly fault-tolerant, extensible platform that answers the true engineering challenge of legged robotics: power management and data distribution.
