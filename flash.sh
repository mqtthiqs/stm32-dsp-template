openocd \
  -f interface/stlink.cfg \
  -c "adapter speed 4000; transport select hla_swd" \
  -f target/stm32f4x.cfg \
  -c "program main.bin verify reset exit 0x08000000"