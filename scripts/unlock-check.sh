#!/usr/bin/env bash
# Report the STM32WLE5's read protection (RDP) level through probe-rs.
#
# FLASH_OPTR (0x58004020, RM0461) holds the RDP byte in effect.  The
# option bytes stored in flash start at 0x1FFF7800 and reach FLASH_OPTR
# only at power-on or OBL_LAUNCH, so reading both tells an unlocked chip
# from one whose unlock is programmed but not loaded yet.  Both stay
# readable at level 1.  Exit status 0 means level 0 is in effect and
# will stay so after a power cycle.
set -euo pipefail

CHIP=STM32WLE5JCIx
FLASH_OPTR=0x58004020
OPTION_BYTES=0x1FFF7800

# rdp_level BYTE: the protection level for an RDP byte given in hex.
# RM0461: 0xAA is level 0, 0xCC level 2, any other value level 1.
rdp_level() {
    case "${1,,}" in
        aa) echo 0 ;;
        cc) echo 2 ;;
        *) echo 1 ;;
    esac
}

# report LIVE STORED: print the verdict for the in-effect and stored
# option words (8 hex digits each); return 0 only for a lasting level 0.
report() {
    local live stored
    live=$(rdp_level "${1: -2}")
    stored=$(rdp_level "${2: -2}")
    if [[ $live != "$stored" ]]; then
        echo "RDP level $live in effect, level $stored programmed (FLASH_OPTR 0x$1, stored 0x$2): power-cycle the board to load it"
        return 1
    fi
    case $live in
        0)
            echo "RDP level 0: unlocked (FLASH_OPTR 0x$1)"
            return 0
            ;;
        1)
            echo "RDP level 1: read-protected (FLASH_OPTR 0x$1): mise run unlock, then power-cycle the board"
            return 1
            ;;
        *)
            echo "RDP level 2: permanently locked (FLASH_OPTR 0x$1)"
            return 1
            ;;
    esac
}

# read_word ADDRESS: one 32-bit word from the target, as 8 hex digits.
read_word() {
    local word
    word=$(probe-rs read --chip "$CHIP" b32 "$1" 1 | tr -d '[:space:]')
    if [[ ! $word =~ ^[0-9a-fA-F]{8}$ ]]; then
        echo "unexpected probe-rs output for $1: '$word'" >&2
        return 1
    fi
    echo "$word"
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
    live=$(read_word "$FLASH_OPTR")
    stored=$(read_word "$OPTION_BYTES")
    report "$live" "$stored"
fi
