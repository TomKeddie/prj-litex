litex_server --uart --uart-port /dev/ttyUSB3 &
~/openocd_riscv/bin/openocd
# riscv64-unknown-elf-gdb -ex 'target remote localhost:3333' soc_soccore_icebreaker/software/bios/bios.elf
