#!/bin/bash
# Host setup for the robot (CM5) and the Pi 5 test box, Pi OS. Idempotent: rerun after changes.
#   sudo software/host/setup.sh                  # can0 (real bus), stack + vision
#   sudo CAN=vcan0 PROFILES=sim,vision software/host/setup.sh   # test box: simulated legs
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

command -v docker >/dev/null || apt-get install -y docker.io docker-compose
usermod -aG docker "$USER_NAME"

# Docker's default open-files limit (~2^30) makes LTTng, loaded by every ROS 2 process,
# allocate 128 MB each. Fix it for every container, not just the ones in compose.yaml.
mkdir -p /etc/docker
cat > /etc/docker/daemon.json <<'J'
{
  "default-ulimits": { "nofile": { "Name": "nofile", "Soft": 65536, "Hard": 65536 } },
  "log-driver": "journald"
}
J
systemctl restart docker

cat > /etc/vector.env <<E
CAN_INTERFACE=$CAN
COMPOSE_PROFILES=$PROFILES
CAMERA_WIDTH=$CAMERA_WIDTH
CAMERA_HEIGHT=$CAMERA_HEIGHT
E

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

systemctl daemon-reload
systemctl daemon-reexec
systemctl enable --now vector-can.service vector-camera.service vector-stack.service
echo "done: CAN=$CAN, profiles=$PROFILES; status: systemctl status vector-stack"
