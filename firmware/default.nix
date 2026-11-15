# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2026, Alec Dusheck
#
# The LT7911EXC bridge firmware: every *.c here, built with SDCC into
# $out/lt7911exc_fw.bin (with the linker's fw.map).
#
# Memory layout: the linker places every variable (vars.h). IRAM holds the
# register bank, the __data variables, SDCC's own data and bits, then the
# stack up to 0xFF. XRAM starts at 0x0001: nothing may sit at 0x0000, whose
# address is a null pointer (the config getters test theirs). CODE must end
# at or before 0xFFFC, where the updater writes its CRC.
#
# Interrupt safety: the ISRs call functions the main loop also runs. Model
# large keeps locals, parameters and register spills at fixed addresses,
# so those shared functions are __reentrant (fw.h), and so is any function
# whose register spills would otherwise take IRAM (SDCC puts spills in
# DSEG). --nooverlay keeps an ISR's locals from sharing bytes with the
# main loop's. The 16/32-bit and float helpers come from SDCC's reentrant
# (stack-auto) libraries (--int-long-reent/--float-reent); the rest of the
# library (memcpy, memset) is not reentrant, so code reachable from an ISR
# copies with copy_bytes() (util.c) instead.
#
# Version and identity come from `env` (build.env's format) as -D flags:
# FW_VERSION a.b.c as FW_VERSION_0..2; EDID_NAME and EDID_MANUFACTURER as
# KEY_0..KEY_12, the text's bytes then a newline and spaces (EDID descriptor
# text, at most 13 characters); EDID_PRODUCT, EDID_SERIAL, USB_VID, USB_PID
# and USB_BCD as numbers (decimal or 0x hex, 16-bit; EDID_SERIAL 32-bit).
# FW_VERSION parts are decimal 0..255. A missing or bad value stops the build.
{
  lib,
  stdenvNoCC,
  sdcc,
  env,
}:

stdenvNoCC.mkDerivation {
  name = "lt7911exc-fw";
  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.fileFilter (
      f: f.hasExt "c" || f.hasExt "h" || f.name == "peep.def" || f.name == "stack.awk"
    ) ./.;
  };
  nativeBuildInputs = [ sdcc ];

  buildPhase = ''
    runHook preBuild

    . ${env}
    defs=
    # KEY WORDS: -DKEY_0, -DKEY_1, ... one per word.
    list() {
        i=0
        for n in $2; do
            defs="$defs -D$1_$i=$n"
            i=$((i + 1))
        done
    }
    # KEY: its value's bytes, then a newline and spaces, in 13 (EDID text).
    text() {
        eval "val=\''${$1:?$1 is required}"
        [ ''${#val} -le 13 ] || { echo "error: $1 is over 13 characters" >&2; exit 1; }
        list $1 "$(printf '%s\n            ' "$val" | head -c 13 | od -An -tu1)"
    }
    # KEY MAX: a decimal (no leading zero, so never octal) or 0x hex number
    # up to MAX, passed as is.
    num() {
        eval "val=\''${$1:?$1 is required}"
        echo "$val" | grep -Eqx '0|[1-9][0-9]{0,9}|0[xX][0-9a-fA-F]{1,8}' && [ $((val)) -le $(($2)) ] ||
            { echo "error: $1 is not a number up to $2: $val" >&2; exit 1; }
        defs="$defs -D$1=$val"
    }
    part='(25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9]?[0-9])'
    echo "''${FW_VERSION:?FW_VERSION is required}" | grep -Eqx "$part\.$part\.$part" ||
        { echo "error: FW_VERSION is not a.b.c, each 0..255 in decimal: $FW_VERSION" >&2; exit 1; }
    list FW_VERSION "$(echo "$FW_VERSION" | tr . ' ')"
    text EDID_NAME
    text EDID_MANUFACTURER
    for key in USB_VID USB_PID USB_BCD EDID_PRODUCT; do
        num $key 0xFFFF
    done
    num EDID_SERIAL 0xFFFFFFFF

    code_end=0xFFFC
    iram_size=0x100
    cflags="-mmcs51 --model-large --std-c99 --opt-code-size --nooverlay --int-long-reent --float-reent --peep-file peep.def"
    lib=${sdcc}/share/sdcc/lib

    rels=
    for src in *.c; do
        sdcc $cflags $defs -c "$src"
        rels="$rels ''${src%.c}.rel"
    done

    sdcc $cflags --nostdlib --code-size $code_end \
        --iram-size $iram_size \
        --xram-loc 0x0001 --xram-size 0x1FFF \
        $rels \
        $lib/large-stack-auto/libint.lib $lib/large-stack-auto/liblong.lib \
        $lib/large-stack-auto/libfloat.lib $lib/large/libsdcc.lib \
        $lib/large/mcs51.lib -o fw.ihx
    makebin -p -s 65536 fw.ihx fw.bin

    size=$(wc -c < fw.bin)
    if [ "$size" -gt $((code_end)) ]; then
        echo "error: fw.bin is $size bytes, over the $((code_end))-byte limit (CRC word at $code_end)" >&2
        exit 1
    fi
    printf 'fw.bin: %d bytes (0x%04X), %d free\n' "$size" "$size" $((code_end - size))

    # Worst-case stack depth (stack.awk) within the IRAM above the reset SP
    # (SDCC's crtstart: the linker's __start__stack - 1). The library modules fw.map lists are rebuilt from SDCC's own
    # sources for their assembly, each matching the linked one byte for byte.
    # INT0 is high priority (int0_enable sets PX0), INT1 and Timer 2 low;
    # ext2 (vector 0x43) is low if SFR_F8 is its priority register, which is
    # not proven, so it counts at either level.
    mkdir stk
    while read l rel; do
        m=''${rel%.rel}
        case $l in */large-stack-auto/*) f=--stack-auto ;; *) f= ;; esac
        if [ -f $lib/src/$m.c ]; then
            (cd stk && sdcc -mmcs51 --model-large $f -c $lib/src/$m.c)
        else
            cp $lib/src/mcs51/$m.asm stk/ && (cd stk && sdas8051 -plosgff $m.asm)
        fi
        if [ "$(sdar p $l $rel | grep '^[TAR] ')" != "$(grep '^[TAR] ' stk/$rel)" ]; then
            echo "error: stack: $m built from source differs from $l's" >&2
            exit 1
        fi
    done < <(awk '/^\/.*\.lib$/ { l = $1 } /\[ .*\.rel \]/ && l { print l, $2; l = "" }' fw.map)
    # The margin covers a slip in the model (every linked instruction is
    # counted), at about one more call level: return address, a 4-byte
    # argument and two saved registers.
    sp=$((0x$(awk '$2 == "__start__stack" { print $1 }' fw.map) - 1))
    awk -v avail=$((iram_size - 1 - sp)) -v margin=8 -v lo="_int1_isr _timer2_isr _ext2_isr" \
        -v hi="_int0_isr _ext2_isr" -f stack.awk *.asm stk/*.asm

    runHook postBuild
  '';

  installPhase = ''
    install -Dm644 fw.bin $out/lt7911exc_fw.bin
    install -Dm644 fw.map $out/fw.map
  '';

  meta.license = lib.licenses.gpl2Only;
}
