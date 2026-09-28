#!/usr/bin/env bash
# First flash over SWD: bootloader + leg firmware, then write-protect the bootloader.
#   flash_swd.sh build/leg-stm32        side board: asks for each of the three cells in turn
#   flash_swd.sh build/nucleo 1         one chip
# Later updates go over CAN (leg_config.py flash). Needs STM32CubeProgrammer's CLI.
set -euo pipefail

dir=${1:?usage: flash_swd.sh BUILD_DIR [CELLS]}
cells=${2:-3}
cli=${STM32_PROGRAMMER_CLI:-STM32_Programmer_CLI}
for f in bootloader.bin leg-node.bin; do
    [[ -f $dir/$f ]] || { echo "$dir/$f not found, build first"; exit 1; }
done

# WRP area A over pages 0-7 (the 16 KB bootloader); start > end switches it off
protect=(-ob WRP1A_STRT=0x0 WRP1A_END=0x7)
unprotect=(-ob WRP1A_STRT=0x7F WRP1A_END=0x0)

for ((i = 1; i <= cells; i++)); do
    if ((cells > 1)); then
        read -rp "cell $i of $cells: Tag-Connect on, board powered, Enter to flash "
    fi
    "$cli" -c port=SWD mode=UR -q "${unprotect[@]}"
    "$cli" -c port=SWD mode=UR -q -e all \
        -w "$dir/bootloader.bin" 0x08000000 -v \
        -w "$dir/leg-node.bin" 0x08004000 -v
    "$cli" -c port=SWD mode=UR -q "${protect[@]}" -rst
    echo "cell $i done"
done
echo "check the node IDs: leg_config.py --channel can0 status"
