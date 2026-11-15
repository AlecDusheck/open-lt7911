# open-lt7911

A open source implementation of the firmware for the Lontium LT7911EXC USB-C/DisplayPort to MIPI CSI-2 video bridge, written in C for the chip's 8051
core and built with [SDCC](https://sdcc.sourceforge.net/).

This project provides a firmware reimplementation based on the LT7911EXC's hardware architecture, register interfaces, and expected device behavior. It is intended to serve as a functional, understandable, and modifiable reference for operating the chip without relying on the manufacturer's firmware.

It receives DisplayPort over USB-C (DP Alt Mode, with USB Power Delivery on the data port and pass-through from a
second power port), serves the EDID, and sends the video out over MIPI. A host talks to it over I2C: the status
registers, the interrupt pin and the flash layout are described in `firmware/regs.h`.

The implementation has been tested and validated on LT7911EXC hardware. While it provides the core functionality required to operate the device, its behavior and feature coverage may differ from the manufacturer's firmware. It is intended as a reference implementation rather than a drop-in, feature-identical replacement.

## Build

With [Nix](https://nixos.org/):

    nix build            # result/lt7911exc_fw.bin and lt7911exc_fw.env (sha256, CRC, version)
    nix build .#firmware # the image and the linker map only
    nix develop          # a shell with SDCC

The build fails if the image does not fit the flash or if the worst-case stack depth nears the top of IRAM
(`firmware/default.nix`).

`build.env` holds the firmware version and the default identity (USB IDs, EDID manufacturer, product, serial and
monitor name). Another flake can build with its own identity through `lib.firmware { pkgs, edid, usb }`.

## Licence

GPL-2.0-only; see [LICENSE](LICENSE). The image links SDCC's runtime library, which is GPL with a linking exception.
