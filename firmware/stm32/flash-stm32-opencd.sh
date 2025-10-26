#!/usr/bin/env bash
set -euo pipefail

ELF="${1:-}"
if [[ -z "${ELF}" || ! -f "${ELF}" ]]; then
  echo "Usage: $0 <path-to-elf>"
  echo "Error: ELF file not found: ${ELF:-<empty>}"
  exit 1
fi

openocd \
  -f interface/stlink.cfg \
  -f target/stm32h7x.cfg \
  -c "adapter speed 4000" \
  -c "init" \
  -c "reset halt" \
  -c "program \"${ELF}\" verify" \
  -c "reset run" \
  -c "shutdown"
