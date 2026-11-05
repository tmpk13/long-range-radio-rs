#!/usr/bin/env bash
# Verdicts of unlock-check.sh for each pairing of in-effect and stored
# RDP bytes, without hardware.
source "$(dirname "$0")/unlock-check.sh"

fails=0

# expect STATUS TEXT LIVE STORED: report LIVE STORED returns STATUS and
# prints a line containing TEXT.
expect() {
    local out status
    out=$(report "$3" "$4") && status=0 || status=$?
    if [[ $status != "$1" || $out != *"$2"* ]]; then
        echo "FAIL: report $3 $4 returned $status: $out"
        fails=$((fails + 1))
    fi
}

expect 0 "level 0: unlocked" 3ffff0aa 3ffff0aa
# A factory Wio-E5.
expect 1 "mise run unlock" 3ffff0bb 3ffff0bb
# After mise run unlock, before the power cycle.
expect 1 "level 0 programmed" 3ffff0bb 3ffff0aa
# A relock programmed but not loaded: unlocked now, locked after power-up.
expect 1 "level 1 programmed" 3FFFF0AA 3ffff0bb
# Any byte other than 0xAA and 0xCC is level 1.
expect 1 "RDP level 1" 3ffff000 3ffff000
expect 1 "permanently locked" 3ffff0cc 3ffff0cc

if ((fails)); then
    exit 1
fi
echo "unlock-check: all verdicts as expected"
