#!/usr/bin/env bash
# mem-report.sh — STM32H753 firmware memory footprint analysis
#
# Parses the release ELF's section sizes, computes flash + RAM usage against the
# H753's actual memory capacities, reports headroom, and emits a markdown table
# suitable for pasting into the README.
#
# Usage:
#   ./mem-report.sh                      # auto-finds the release binary
#   ./mem-report.sh path/to/elf          # explicit binary
#   ./mem-report.sh --md                 # markdown-table output only (for README)
#
# Run from firmware/stm32/ (or pass an explicit ELF path).

set -euo pipefail

# ── STM32H753 memory map (adjust if your linker script partitions differently) ──
# The H753 has 2 MB flash and 1 MB SRAM total, split across banks/domains:
#   DTCM  128 KB   (fast, tightly-coupled — often the main stack + hot data)
#   AXI   512 KB   (D1 domain SRAM)
#   SRAM1 128 KB / SRAM2 128 KB / SRAM3 32 KB  (D2 domain)
#   SRAM4 64 KB    (D3 domain)
# For a headline "RAM" number we use the region your linker actually places
# .data/.bss/stack into. Set these to match your memory.x / linker script.
FLASH_TOTAL_KB=2048
RAM_TOTAL_KB=512      # <-- set to the RAM region your linker uses for .data+.bss+stack
                      #     (e.g. the AXI SRAM region in memory.x). Change to match!
STACK_BUDGET_KB=32    # <-- the stack size your linker reserves (from _stack_size / memory.x)

TARGET_TRIPLE="thumbv7em-none-eabihf"

# ── locate the size tool (prefer llvm via rustup, fall back to arm-none-eabi) ──
find_size_tool() {
    if command -v rust-size >/dev/null 2>&1; then echo "rust-size"; return; fi
    # llvm-size shipped with the llvm-tools-preview component
    local llvm_size
    llvm_size="$(find "$(rustc --print sysroot)" -name 'llvm-size' 2>/dev/null | head -1 || true)"
    if [ -n "$llvm_size" ]; then echo "$llvm_size"; return; fi
    if command -v arm-none-eabi-size >/dev/null 2>&1; then echo "arm-none-eabi-size"; return; fi
    echo ""; return
}

SIZE_TOOL="$(find_size_tool)"
if [ -z "$SIZE_TOOL" ]; then
    echo "ERROR: no size tool found. Install one of:" >&2
    echo "  rustup component add llvm-tools-preview   (then: cargo install cargo-binutils)" >&2
    echo "  or your ARM GCC toolchain (arm-none-eabi-size)" >&2
    exit 1
fi

# ── find the ELF ──
MD_ONLY=0
ELF=""
for arg in "$@"; do
    case "$arg" in
        --md) MD_ONLY=1 ;;
        *) ELF="$arg" ;;
    esac
done

if [ -z "$ELF" ]; then
    # auto-find: newest executable ELF in the release dir that isn't a .d/.rlib
    REL_DIR="target/${TARGET_TRIPLE}/release"
    if [ ! -d "$REL_DIR" ]; then
        echo "ERROR: $REL_DIR not found. Build first: cargo build --release" >&2
        echo "Or pass an explicit ELF path." >&2
        exit 1
    fi
    # pick the largest file with no extension (the linked binary), excluding deps/
    ELF="$(find "$REL_DIR" -maxdepth 1 -type f ! -name '*.*' -printf '%s %p\n' 2>/dev/null \
           | sort -rn | head -1 | cut -d' ' -f2- || true)"
    if [ -z "$ELF" ]; then
        echo "ERROR: could not auto-find the release binary in $REL_DIR." >&2
        echo "Pass the ELF path explicitly: ./mem-report.sh $REL_DIR/<name>" >&2
        exit 1
    fi
fi

if [ ! -f "$ELF" ]; then
    echo "ERROR: ELF not found: $ELF" >&2
    exit 1
fi

# ── parse sizes (System V / -A format gives per-section) ──
# We want: .text + .rodata (+ .vector_table etc.) → FLASH
#          .data (init'd RAM, also counts against flash for the load image)
#          .bss  (zero-init RAM)
SIZE_OUT="$("$SIZE_TOOL" -A "$ELF" 2>/dev/null)"

get_section() {
    # $1 = section name; prints size in bytes (0 if absent)
    echo "$SIZE_OUT" | awk -v s="$1" '$1==s {print $2; found=1} END{if(!found) print 0}'
}

TEXT=$(get_section ".text")
RODATA=$(get_section ".rodata")
DATA=$(get_section ".data")
BSS=$(get_section ".bss")
VECTORS=$(get_section ".vector_table")
# some setups name it differently; sum a few common flash-resident extras
UNINIT=$(get_section ".uninit")

# FLASH image = text + rodata + vectors + data (data's init values live in flash)
FLASH_BYTES=$(( TEXT + RODATA + VECTORS + DATA ))
# Static RAM = data + bss (the compile-time-known RAM; stack is separate/dynamic)
STATIC_RAM_BYTES=$(( DATA + BSS ))
# RAM including the reserved stack budget (upper bound on steady-state RAM)
RAM_WITH_STACK_BYTES=$(( STATIC_RAM_BYTES + STACK_BUDGET_KB * 1024 ))

FLASH_TOTAL=$(( FLASH_TOTAL_KB * 1024 ))
RAM_TOTAL=$(( RAM_TOTAL_KB * 1024 ))

pct() { awk -v a="$1" -v b="$2" 'BEGIN{ if(b==0){print "0.0"} else {printf "%.1f", (a/b)*100} }'; }
kb()  { awk -v b="$1" 'BEGIN{ printf "%.2f", b/1024 }'; }

FLASH_PCT=$(pct "$FLASH_BYTES" "$FLASH_TOTAL")
STATIC_RAM_PCT=$(pct "$STATIC_RAM_BYTES" "$RAM_TOTAL")
RAM_STACK_PCT=$(pct "$RAM_WITH_STACK_BYTES" "$RAM_TOTAL")

if [ "$MD_ONLY" -eq 1 ]; then
    # markdown table for the README
    cat << MD
| Resource | Used | Capacity | Utilization |
| --- | --- | --- | --- |
| **Flash** (.text + .rodata + .data init) | $(kb $FLASH_BYTES) KB | ${FLASH_TOTAL_KB} KB | ${FLASH_PCT}% |
| **Static RAM** (.data + .bss) | $(kb $STATIC_RAM_BYTES) KB | ${RAM_TOTAL_KB} KB | ${STATIC_RAM_PCT}% |
| **RAM + reserved stack** (${STACK_BUDGET_KB} KB) | $(kb $RAM_WITH_STACK_BYTES) KB | ${RAM_TOTAL_KB} KB | ${RAM_STACK_PCT}% |

<sub>Static RAM is compile-time-known (.data + .bss); stack is a reserved budget (peak stack usage is measured separately via watermarking — see WCET/stack instrumentation).</sub>
MD
    exit 0
fi

# ── human-readable report ──
echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  STM32H753 Firmware Memory Report"
echo "══════════════════════════════════════════════════════════════"
echo "  Binary: $ELF"
echo "  Size tool: $SIZE_TOOL"
echo "──────────────────────────────────────────────────────────────"
printf "  %-28s %12s\n" "Section" "Bytes"
printf "  %-28s %12s\n" ".text (code)"            "$TEXT"
printf "  %-28s %12s\n" ".rodata (const data)"    "$RODATA"
printf "  %-28s %12s\n" ".vector_table"           "$VECTORS"
printf "  %-28s %12s\n" ".data (init'd RAM)"      "$DATA"
printf "  %-28s %12s\n" ".bss (zero'd RAM)"       "$BSS"
echo "──────────────────────────────────────────────────────────────"
echo "  FLASH"
printf "    Used:        %8s KB  (text+rodata+vectors+data-init)\n" "$(kb $FLASH_BYTES)"
printf "    Capacity:    %8s KB\n" "$FLASH_TOTAL_KB"
printf "    Utilization: %8s %%\n" "$FLASH_PCT"
printf "    Headroom:    %8s KB\n" "$(kb $(( FLASH_TOTAL - FLASH_BYTES )))"
echo "──────────────────────────────────────────────────────────────"
echo "  RAM"
printf "    Static (.data+.bss):   %8s KB\n" "$(kb $STATIC_RAM_BYTES)"
printf "    + reserved stack:      %8s KB  (%s KB stack budget)\n" "$(kb $RAM_WITH_STACK_BYTES)" "$STACK_BUDGET_KB"
printf "    Capacity:              %8s KB\n" "$RAM_TOTAL_KB"
printf "    Static utilization:    %8s %%\n" "$STATIC_RAM_PCT"
printf "    With-stack util:       %8s %%\n" "$RAM_STACK_PCT"
printf "    Headroom (w/ stack):   %8s KB\n" "$(kb $(( RAM_TOTAL - RAM_WITH_STACK_BYTES )))"
echo "══════════════════════════════════════════════════════════════"
echo ""
echo "  NOTE: RAM_TOTAL_KB ($RAM_TOTAL_KB) and STACK_BUDGET_KB ($STACK_BUDGET_KB) are set at"
echo "  the top of this script — make them match your memory.x / linker regions."
echo "  Peak *actual* stack usage is dynamic; measure it via stack watermarking"
echo "  (paint the stack with a sentinel, run the worst-case path, find the"
echo "  high-water mark). Static analysis here covers .data/.bss only."
echo ""
