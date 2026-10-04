#!/bin/bash
# Host setup for the robot (CM5) and the Pi 5 test box, Pi OS. Idempotent: rerun after changes.
#   sudo software/host/setup.sh                  # can0 (real bus), stack + vision
#   sudo CAN=vcan0 PROFILES=sim,vision software/host/setup.sh   # test box: simulated legs
#   GNSS=/dev/ttyAMA0 (default; "" = none), NAV=true for the EKFs and the waypoint follower
#   LINKS=true (default with can0) for the link manager; ELRS=/dev/ttyAMA2 (UART2, default;
#   "" = none); LTE_AT=/dev/ttyUSB2 for the modem's signal ("" = default, don't ask it)
#   On the carrier: IO=/dev/ttyAMA2 (the IO MCU on UART2) with ELRS=io and GNSS="", IMU=true
# Installs: Docker (with a sane open-files limit), CAN bring-up, the camera service, the
# stack as a systemd service, and the hardware watchdog. Images are built separately.
set -euo pipefail
[[ $EUID == 0 ]] || { echo "run with sudo"; exit 1; }

REPO=$(cd "$(dirname "$0")/../.." && pwd)
CAN=${CAN:-can0}
PROFILES=${PROFILES:-vision}
# capture size: YOLO runs on a 320 px copy, masks and the stream use the full frame
CAMERA_WIDTH=${CAMERA_WIDTH:-1280}
CAMERA_HEIGHT=${CAMERA_HEIGHT:-960}
USER_NAME=${SUDO_USER:-pi}
GNSS=${GNSS-/dev/ttyAMA0}
NAV=${NAV:-false}
LINKS=${LINKS:-$([[ $CAN == can0 ]] && echo true || echo false)}
ELRS=${ELRS-/dev/ttyAMA2}
LTE_AT=${LTE_AT:-}
IO=${IO:-}
IMU=${IMU:-false}

command -v docker >/dev/null || apt-get install -y docker.io docker-compose
usermod -aG docker "$USER_NAME"

# Docker's default open-files limit (~2^30) makes LTTng, loaded by every ROS 2 process,
# allocate 128 MB each. Fix it for every container, not just the ones in compose.yaml.
mkdir -p /etc/docker
DAEMON='{
  "default-ulimits": { "nofile": { "Name": "nofile", "Soft": 65536, "Hard": 65536 } },
  "log-driver": "journald"
}'
# only when it changes: a restart under running containers can race the next compose up
if [[ "$(cat /etc/docker/daemon.json 2>/dev/null)" != "$DAEMON" ]]; then
    echo "$DAEMON" > /etc/docker/daemon.json
    systemctl restart docker
fi

cat > /etc/vector.env <<E
CAN_INTERFACE=$CAN
COMPOSE_PROFILES=$PROFILES
CAMERA_WIDTH=$CAMERA_WIDTH
CAMERA_HEIGHT=$CAMERA_HEIGHT
GNSS_PORT=$GNSS
NAV=$NAV
LINKS=$LINKS
ELRS_PORT=$ELRS
LTE_AT_PORT=$LTE_AT
IO_PORT=$IO
IMU=$IMU
E

# UART0 on GPIO14 (TX) / 15 (RX) for the GNSS receiver, /dev/ttyAMA0 after a reboot. The
# Pi 5 / CM5 console lives on its own debug UART, so nothing else is on these pins.
BOOT_CFG=/boot/firmware/config.txt
if [[ -n $GNSS && -f $BOOT_CFG ]] && ! grep -q '^dtparam=uart0=on' "$BOOT_CFG"; then
    echo 'dtparam=uart0=on' >> "$BOOT_CFG"
    echo "UART0 enabled in $BOOT_CFG: reboot for /dev/ttyAMA0"
fi
# UART2 on GPIO4 (TX) / 5 (RX) for the ExpressLRS receiver, /dev/ttyAMA2 (Pi 5 / CM5 overlay)
if [[ ( -n $IO || ( -n $ELRS && $ELRS != io ) ) && -f $BOOT_CFG ]] && ! grep -q '^dtoverlay=uart2-pi5' "$BOOT_CFG"; then
    echo 'dtoverlay=uart2-pi5' >> "$BOOT_CFG"
    echo "UART2 enabled in $BOOT_CFG: reboot for /dev/ttyAMA2"
fi

# The A7670E (RNDIS / ECM) shows up as a wired interface, which NetworkManager would put
# ahead of Wi-Fi. Match it by driver and give it a higher route metric: Wi-Fi first, LTE
# when there's no Wi-Fi.
if [[ -d /etc/NetworkManager/system-connections ]]; then
    cat > /etc/NetworkManager/system-connections/vector-lte.nmconnection <<'N'
[connection]
id=vector-lte
type=ethernet
autoconnect=true

[match]
driver=rndis_host;cdc_ether;cdc_ncm;

[ipv4]
method=auto
route-metric=700

[ipv6]
method=auto
route-metric=700
N
    chmod 600 /etc/NetworkManager/system-connections/vector-lte.nmconnection
    nmcli connection reload 2>/dev/null || true
fi

# I2C1 (GPIO2/3) for the nose board (VL53L8CX, compass) at 400 kHz, as /dev/i2c-1
if [[ -f $BOOT_CFG ]] && ! grep -q '^dtparam=i2c_arm=on' "$BOOT_CFG"; then
    echo 'dtparam=i2c_arm=on,i2c_arm_baudrate=400000' >> "$BOOT_CFG"
    echo "I2C enabled in $BOOT_CFG: reboot for /dev/i2c-1"
fi
echo i2c-dev > /etc/modules-load.d/i2c-dev.conf

cat > /usr/local/bin/vector-can-up <<'S'
#!/bin/bash
# vcan for testing without hardware, otherwise the real bus at 1 Mbit/s
set -e
. /etc/vector.env
if [[ $CAN_INTERFACE == vcan* ]]; then
    modprobe vcan
    ip link show "$CAN_INTERFACE" >/dev/null 2>&1 || ip link add dev "$CAN_INTERFACE" type vcan
else
    ip link set "$CAN_INTERFACE" down 2>/dev/null || true
    ip link set "$CAN_INTERFACE" type can bitrate 1000000 restart-ms 100
fi
ip link set "$CAN_INTERFACE" up
S
chmod +x /usr/local/bin/vector-can-up

cat > /etc/systemd/system/vector-can.service <<U
[Unit]
Description=Leg bus (CAN) up
Before=vector-stack.service
[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/vector-can-up
[Install]
WantedBy=multi-user.target
U

# The camera stays on the host (native libcamera) and serves raw frames to the vision
# container; rpicam-vid exits when its reader goes, so systemd restarts it.
cat > /etc/systemd/system/vector-camera.service <<U
[Unit]
Description=Camera raw frames on tcp://127.0.0.1:9000
[Service]
User=$USER_NAME
EnvironmentFile=/etc/vector.env
ExecStart=/usr/bin/rpicam-vid -t 0 -n --width \${CAMERA_WIDTH} --height \${CAMERA_HEIGHT} --framerate 15 --codec yuv420 --listen -o tcp://127.0.0.1:9000
Restart=always
RestartSec=1
[Install]
WantedBy=multi-user.target
U

cat > /etc/systemd/system/vector-stack.service <<U
[Unit]
Description=Robot stack (docker compose)
Requires=docker.service vector-can.service
After=docker.service vector-can.service network-online.target
[Service]
Type=oneshot
RemainAfterExit=yes
EnvironmentFile=/etc/vector.env
WorkingDirectory=$REPO/software/docker
ExecStart=/usr/bin/docker compose up -d
ExecStop=/usr/bin/docker compose down
TimeoutStartSec=300
[Install]
WantedBy=multi-user.target
U

# Hardware watchdog: a hung system reboots; the legs crouch on their own meanwhile.
mkdir -p /etc/systemd/system.conf.d
cat > /etc/systemd/system.conf.d/vector-watchdog.conf <<'W'
[Manager]
RuntimeWatchdogSec=15
RebootWatchdogSec=2min
W

# The SD card: fewer writes, and a quick recovery when power goes anyway. Root stays
# writable (Docker's overlay2 can't sit on an overlay root without its own partition);
# the power board gives the CM5 a clean halt.
mkdir -p /etc/systemd/journald.conf.d
cat > /etc/systemd/journald.conf.d/vector.conf <<'J'
[Journal]
Storage=volatile
RuntimeMaxUse=64M
J
# swap only in zram (the default adds a writeback file on the card); /tmp is tmpfs already
if [[ -d /etc/rpi ]]; then
    mkdir -p /etc/rpi/swap.conf.d
    printf '[Main]\nMechanism=zram\n' > /etc/rpi/swap.conf.d/vector.conf
fi
grep -q 'fsck.repair=yes' /boot/firmware/cmdline.txt 2>/dev/null || sed -i 's/$/ fsck.repair=yes/' /boot/firmware/cmdline.txt

systemctl daemon-reload
systemctl daemon-reexec
systemctl enable --now vector-can.service vector-camera.service
systemctl enable vector-stack.service
# the images come from firstboot.sh or a manual build; without them compose would try Docker Hub
if docker image inspect vector >/dev/null 2>&1; then
    systemctl restart vector-stack.service
else
    echo "no vector image yet: build it, then systemctl start vector-stack"
fi
echo "done: CAN=$CAN, profiles=$PROFILES, GNSS=${GNSS:-none}, nav=$NAV, links=$LINKS, ELRS=${ELRS:-none}, IO=${IO:-none}, IMU=$IMU; status: systemctl status vector-stack"
