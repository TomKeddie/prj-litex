litex_server --udp --udp-ip 192.168.1.50 --udp-port 20000 &
~/git/SpinalHDL/openocd_riscv/src/openocd
# riscv64-unknown-elf-gdb -ex 'target remote localhost:3333' soc_etherbonesoc_arty/software/bios/bios.elf 
