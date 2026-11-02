#!/usr/bin/env python3
"""Flash the carrier's IO MCU from the CM5 through ST's ROM bootloader on its USART1.

  io_flash.py io-node.bin [--port /dev/ttyAMA2]

With io_bridge running, asks it (io/bootloader) to reset the MCU into the bootloader and let
go of the port; without it, sends the bootloader message itself. Then vector_io.stm32boot
erases, writes, verifies and starts the image.
"""
import argparse
import sys
import time

import serial

from vector_io import link, stm32boot


def via_bridge(timeout=3.0):
    try:
        import rclpy
        from std_srvs.srv import Trigger
    except ImportError:
        return False
    rclpy.init()
    node = rclpy.create_node('io_flash')
    try:
        cli = node.create_client(Trigger, 'io/bootloader')
        if not cli.wait_for_service(timeout_sec=timeout):
            return False
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None or not res.success:
            print('io_bridge:', res.message if res else 'no answer')
            return False
        print('io_bridge:', res.message)
        return True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def direct(port):
    with serial.Serial(port, 1000000, timeout=0.1) as s:
        s.write(link.encode(link.BOOTLOADER, link.BOOT_MAGIC.to_bytes(4, 'little')))
        s.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('image')
    ap.add_argument('--port', default='/dev/ttyAMA2')
    ap.add_argument('--baud', type=int, default=115200, help='bootloader baud rate (autobaud, up to 115200)')
    a = ap.parse_args()
    with open(a.image, 'rb') as f:
        image = f.read()

    def rom():
        return serial.Serial(a.port, a.baud, parity=serial.PARITY_EVEN, timeout=0.5)

    # an earlier update cut short leaves the ROM bootloader running (or starting at power-on):
    # ask it first, since a 1 Mbaud message would make a fresh one lock onto the wrong baud rate
    with rom() as s:
        try:
            stm32boot.Bootloader(s).sync(tries=1)
            synced = True
            print('ROM bootloader already running')
        except stm32boot.BootError:
            synced = False
    if not synced:
        if not via_bridge():
            print(f'no io_bridge, sending the bootloader request on {a.port}')
            direct(a.port)
        time.sleep(0.3)
    with rom() as s:
        try:
            stm32boot.flash(stm32boot.Bootloader(s), image, synced=synced)
        except stm32boot.BootError as e:
            sys.exit(f'io_flash: {e}')
    print('started')


if __name__ == '__main__':
    main()
