#!/bin/bash
# A robot SD card from the official Raspberry Pi OS Lite (arm64) image, set up to install
# itself on first boot (cloud-init, then firstboot.sh: about 20 minutes, needs internet).
#
#   software/image/make_card.sh vector.img --ssid Home            an image file, no root needed
#   sudo software/image/make_card.sh /dev/sdX --ssid Home          straight to a card
#
#   --ssid NAME        Wi-Fi to join (asks for the password; without it: Ethernet only)
#   --ssh-key FILE     public key for user pi (default ~/.ssh/id_ed25519.pub, then id_rsa.pub)
#   --hostname NAME    default vector (reachable as vector.local)
#   --can IF           can0 (default) or vcan0 for a test box
#   --profiles LIST    compose profiles, default vision; sim,vision for a test box
#   --os FILE          a local .img.xz instead of the latest download
#
# The card gets this checkout's firmware/, protocol/, software/ and docs/ (as they are now,
# local changes included; VERSION says which), so nothing is fetched from the private repo.
# The Wi-Fi password ends up on the card's boot partition, like with Raspberry Pi Imager.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
target=${1:?usage: make_card.sh IMAGE_FILE|/dev/sdX [options]}
shift
ssid= key= host=vector can=can0 profiles=vision os=
while (($#)); do
    case $1 in
        --ssid) ssid=$2; shift 2 ;;
        --ssh-key) key=$2; shift 2 ;;
        --hostname) host=$2; shift 2 ;;
        --can) can=$2; shift 2 ;;
        --profiles) profiles=$2; shift 2 ;;
        --os) os=$2; shift 2 ;;
        *) echo "unknown option $1"; exit 2 ;;
    esac
done
home=$(getent passwd "${SUDO_USER:-$USER}" | cut -d: -f6)
for k in "$key" "$home/.ssh/id_ed25519.pub" "$home/.ssh/id_rsa.pub"; do
    [[ -n $k && -f $k ]] && { key=$k; break; }
done
[[ -f $key ]] || { echo "no SSH public key, pass --ssh-key"; exit 1; }
psk=
if [[ -n $ssid ]]; then
    read -rsp "Wi-Fi password for $ssid: " psk
    echo
fi

# the OS image, verified against the published SHA-256
if [[ -z $os ]]; then
    url=$(curl -sI https://downloads.raspberrypi.com/raspios_lite_arm64_latest | sed -n 's/^location: *//Ip' | tr -d '\r')
    os="$home/.cache/vector/$(basename "$url")"
    mkdir -p "$(dirname "$os")"
    if [[ ! -f $os ]]; then
        curl -L --fail -o "$os.part" "$url"
        (cd "$(dirname "$os")" && curl -sL --fail "$url.sha256" | sed "s/ .*\$/  $(basename "$os").part/" | sha256sum -c -)
        mv "$os.part" "$os"
    fi
fi
echo "OS: $os"

if [[ -b $target ]]; then
    [[ $EUID == 0 ]] || { echo "writing a card needs sudo"; exit 1; }
    lsblk -o NAME,SIZE,MODEL,MOUNTPOINT "$target"
    read -rp "everything on $target will be erased, type yes: " ok
    [[ $ok == yes ]] || exit 1
    umount "$target"?* 2>/dev/null || true
    xz -dc "$os" | dd of="$target" bs=4M conv=fsync status=progress
    partprobe "$target" 2>/dev/null || true
    sleep 2
    boot=$(lsblk -lnpo NAME "$target" | sed -n 2p)
    mt=(-i "$boot")
else
    xz -dc "$os" > "$target"
    start=$(sfdisk -J "$target" | python3 -c 'import json,sys; print(json.load(sys.stdin)["partitiontable"]["partitions"][0]["start"])')
    mt=(-i "$target@@$((start * 512))")
fi

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
esc() { printf '%s' "$1" | sed 's/[\\&|]/\\&/g'; }
sed -e "s|@HOSTNAME@|$(esc "$host")|" -e "s|@SSH_KEY@|$(esc "$(cat "$key")")|" \
    -e "s|@CAN@|$(esc "$can")|" -e "s|@PROFILES@|$(esc "$profiles")|" "$HERE/user-data.in" > "$work/user-data"
echo "instance-id: vector-$(date +%Y%m%d%H%M%S)" > "$work/meta-data"
if [[ -n $ssid ]]; then
    sed -e "s|@SSID@|$(esc "$ssid")|" -e "s|@PSK@|$(esc "$psk")|" "$HERE/network-config.in" > "$work/network-config"
else
    printf 'network:\n  version: 2\n  ethernets:\n    eth0:\n      dhcp4: true\n' > "$work/network-config"
fi
# (LFS filters off: hardware/ has Git LFS files, and git-lfs may not be installed)
(cd "$REPO" && echo "$(git rev-parse --short HEAD)$(git -c filter.lfs.process= -c filter.lfs.clean=cat -c filter.lfs.required=false status --porcelain -- firmware protocol software docs | grep -q . && echo ' + local changes')") > "$work/VERSION"
(cd "$REPO" && git ls-files -co --exclude-standard -- firmware protocol software docs) > "$work/files"
tar -czf "$work/vector.tar.gz" -C "$REPO" -T "$work/files" -C "$work" VERSION
for f in user-data meta-data network-config vector.tar.gz; do
    mcopy -o "${mt[@]}" "$work/$f" "::$f"
done
mdir "${mt[@]}" :: | grep -E 'user-data|network-config|meta-data|vector'
sync
echo "done: $target ($(cat "$work/VERSION")), host $host, CAN $can, profiles $profiles"
echo "first boot installs everything and reboots; follow it with: ssh pi@$host.local tail -f /var/log/vector-firstboot.log"
