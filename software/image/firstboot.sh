#!/bin/bash
# First boot of a card from make_card.sh (cloud-init runs it once), or by hand on any Pi OS
# install with the repo in /home/pi/vector: host setup, the images, the YOLO model, reboot.
# Log: /var/log/vector-firstboot.log. About 20 minutes on a CM5, most of it downloads.
set -euxo pipefail
REPO=/home/pi/vector
cd "$REPO"
cat VERSION 2>/dev/null || true

"$REPO/software/host/setup.sh"

docker build -f software/docker/Dockerfile -t vector .
docker build -f software/vision/Dockerfile --target vision -t vector-vision .
# the ONNX models, made once with PyTorch in a throwaway image, then only the file is kept
docker build -f software/vision/Dockerfile --target export -t vector-yolo-export .
docker run --rm -e SIZES=320 -v docker_models:/models vector-yolo-export
docker rmi vector-yolo-export
docker builder prune -af

touch /var/lib/vector-firstboot-done
reboot
