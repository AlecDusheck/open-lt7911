# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2026, Alec Dusheck
{
  description = "Firmware for the LT7911EXC video bridge";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-26.05";

  outputs =
    { self, nixpkgs }:
    let
      inherit (nixpkgs) lib;
      forAll = f: lib.genAttrs [ "aarch64-darwin" "x86_64-linux" ] (system: f nixpkgs.legacyPackages.${system});

      # The firmware built from `env` (build.env's format), with its lt7911exc_fw.env: IMAGE_SHA; IMAGE_CRC, the
      # CRC-32/MPEG-2 over the file padded with 0xff to 0xfffc bytes, each 32-bit word fed last byte first; IMAGE_VER,
      # what the bridge reports after the flash (E0:81..83).
      image =
        pkgs: env:
        pkgs.runCommand "lt7911exc-fw" { } ''
          . ${env}
          mkdir -p $out
          install -m644 ${pkgs.callPackage ./firmware { inherit env; }}/lt7911exc_fw.bin $out/
          {
            echo "IMAGE_SHA=$(sha256sum $out/lt7911exc_fw.bin | cut -d' ' -f1)"
            { cat $out/lt7911exc_fw.bin; head -c $((0xfffc - $(stat -c %s $out/lt7911exc_fw.bin))) /dev/zero | tr '\0' '\377'; } |
              od -An -v -tu1 |
              awk '{ for (i = 1; i <= NF; i++) w[n++] = $i }
                   END {
                     crc = 0xffffffff
                     for (i = 0; i < n; i += 4)
                       for (j = 3; j >= 0; j--) {
                         crc = xor(crc, lshift(w[i + j], 24))
                         for (b = 0; b < 8; b++)
                           crc = and(and(crc, 0x80000000) ? xor(lshift(crc, 1), 0x04c11db7) : lshift(crc, 1), 0xffffffff)
                       }
                     printf "IMAGE_CRC=0x%08x\n", crc
                   }'
            echo "IMAGE_VER=$FW_VERSION"
          } > $out/lt7911exc_fw.env
        '';
    in
    {
      # The image given an identity; an unknown key in `edid` or `usb` is an error. The version is build.env's
      # FW_VERSION.
      lib.firmware =
        {
          pkgs,
          edid,
          usb,
        }:
        let
          version = lib.findFirst (lib.hasPrefix "FW_VERSION=") (throw "build.env: no FW_VERSION") (
            lib.splitString "\n" (builtins.readFile ./build.env)
          );
          edidEnv =
            {
              monitor_name,
              manufacturer,
              product_code,
              serial_number,
            }:
            ''
              EDID_NAME="${monitor_name}"
              EDID_MANUFACTURER=${manufacturer}
              EDID_PRODUCT=${toString product_code}
              EDID_SERIAL=${toString serial_number}
            '';
          usbEnv =
            {
              vendor_id,
              product_id,
              device_release,
            }:
            ''
              USB_VID=${toString vendor_id}
              USB_PID=${toString product_id}
              USB_BCD=${toString device_release}
            '';
        in
        image pkgs (pkgs.writeText "build.env" (version + "\n" + edidEnv edid + usbEnv usb));

      packages = forAll (pkgs: {
        firmware = pkgs.callPackage ./firmware { env = ./build.env; };
        default = image pkgs ./build.env;
      });

      # The firmware build runs its size and stack checks.
      checks = forAll (pkgs: self.packages.${pkgs.stdenv.hostPlatform.system});

      devShells = forAll (pkgs: {
        default = pkgs.mkShell { packages = [ pkgs.sdcc ]; };
      });
    };
}
