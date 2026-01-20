#!/usr/bin/env bash
set -euo pipefail

ELF="${1:-target/thumbv7em-none-eabihf/release/stm32_firmware}"

if [[ ! -f "${ELF}" ]]; then
  echo "ELF not found: ${ELF}"
  exit 1
fi
echo "[gdb_flash] ELF = ${ELF}"

openocd \
  -f interface/stlink.cfg \
  -f target/stm32h7x.cfg \
  -c "adapter speed 4000" \
  -c "init; reset halt" \
  >/tmp/openocd.log 2>&1 &
OCD_PID=$!
trap 'kill ${OCD_PID} 2>/dev/null || true' EXIT
sleep 0.4

arm-none-eabi-gdb -q \
  -ex "set confirm off" \
  -ex "set pagination off" \
  -ex "set print asm-demangle on" \
  -ex "file ${ELF}" \
  -ex "target extended-remote :3333" \
  -ex "monitor reset halt" \
  -ex "load" \
  -ex "monitor reset halt" \
  -ex "monitor mdw 0xE000ED08 1" \
  -ex "info registers"
