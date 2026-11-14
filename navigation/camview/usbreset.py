#!/usr/bin/env python3
"""USB-reset the RealSense D435i, no sudo needed.

librealsense talks to the camera over libusb, not the kernel uvcvideo driver,
so there are no /dev/video* nodes to reopen. If the driver is killed without
closing cleanly the device can be left mid-stream, and the next launch shows
"control_transfer ... Resource temporarily unavailable" and
"uvc streamer watchdog triggered". A USBDEVFS_RESET clears that.

    python3 ~/.scuba/camview/usbreset.py

Stop every process using the camera FIRST, or the reset will not stick.
"""
import fcntl
import os
import sys

USBDEVFS_RESET = ord('U') << 8 | 20      # _IO('U', 20)
VENDOR, PRODUCT = '8086', '0b3a'         # Intel D435i


def find():
    """(busnum, devnum, syspath) for every matching device."""
    out = []
    root = '/sys/bus/usb/devices'
    for name in sorted(os.listdir(root)):
        d = os.path.join(root, name)
        try:
            with open(os.path.join(d, 'idVendor')) as f:
                v = f.read().strip()
            with open(os.path.join(d, 'idProduct')) as f:
                p = f.read().strip()
        except OSError:
            continue
        if (v, p) != (VENDOR, PRODUCT):
            continue
        try:
            with open(os.path.join(d, 'busnum')) as f:
                bus = int(f.read().strip())
            with open(os.path.join(d, 'devnum')) as f:
                dev = int(f.read().strip())
        except OSError:
            continue
        out.append((bus, dev, d))
    return out


def main():
    devs = find()
    if not devs:
        print('!! no Intel %s:%s on USB - is the camera unplugged?'
              % (VENDOR, PRODUCT))
        return 1
    rc = 0
    for bus, dev, syspath in devs:
        node = '/dev/bus/usb/%03d/%03d' % (bus, dev)
        print('resetting %s  (%s)' % (node, syspath))
        try:
            fd = os.open(node, os.O_WRONLY)
        except OSError as e:
            print('  !! cannot open: %s' % e)
            print('  !! something still has the camera open, or permissions '
                  'changed')
            rc = 1
            continue
        try:
            fcntl.ioctl(fd, USBDEVFS_RESET, 0)
            print('  reset OK')
        except OSError as e:
            print('  !! ioctl failed: %s' % e)
            rc = 1
        finally:
            os.close(fd)
    if rc == 0:
        print('done - the device re-enumerates, so give it ~2 s before '
              'launching')
    return rc


if __name__ == '__main__':
    sys.exit(main())
