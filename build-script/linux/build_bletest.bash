#!/bin/bash

build_path="../../build"
if [ ! -d $build_path ];then
        mkdir -p $build_path
        cd $build_path
else
        rm -rf $build_path
        mkdir -p $build_path
        cd $build_path
fi

echo "=====build start======"
#read -p "please select board name FPGA or 1008: " board_name
#read -p "please select soc name MS1008 or  MS1008_V2: " soc_name
board_name="$1"
soc_name="$2"
input_err=0

if [ "$board_name" != "FPGA" ] && [ "$board_name" != "1008" ]; then
	input_err=1
fi
if [ "$soc_name" != "MS1008" ] && [ "$soc_name" != "MS1008_V2" ]; then
	input_err=1
fi
if [ "$input_err" -eq 1 ]; then
        echo "please select board name FPGA or 1008"
        echo "please select soc name MS1008 or MS1008_V2"
        echo "build error, need board and soc name"
        exit 1
fi

echo "makefile generating"
cmake ../tools/test_ble -G"Unix Makefiles"  -DBOARD=$board_name -DSOC=$soc_name  -DBT_SUPPORT=YES
echo "start make"
make
riscv-nuclei-elf-objdump  -S -d   test_ble_sdk > test_ble_sdk.asm

riscv-nuclei-elf-objcopy -j .func.rom -O binary "test_ble_sdk"  "test_ble_sdk_rom.bin"
riscv-nuclei-elf-objcopy -R .func.rom -O binary "test_ble_sdk"  "test_ble_sdk_flash.bin"
riscv-nuclei-elf-size --format=berkeley "test_ble_sdk"
dd if=../build-script/bootrom.bin of=rom.bin bs=1
dd if=test_ble_sdk_rom.bin of=rom.bin bs=1 seek=4096
hexdump -v -e '/4 "%08X\n"'  "rom.bin" > "rom.hex"
../build-script/linux/ms_hexdump


md5sum test_ble_sdk_rom.bin > md5.txt
md5sum rom.bin >> md5.txt
md5sum rom.hex >> md5.txt
md5sum rom_part0.bin >> md5.txt
md5sum rom_part0.hex >> md5.txt
md5sum rom_part1.bin >> md5.txt
md5sum rom_part1.hex >> md5.txt
md5sum rom_part2.bin >> md5.txt
md5sum rom_part2.hex >> md5.txt
md5sum rom_part3.bin >> md5.txt
md5sum rom_part3.hex >> md5.txt

echo "build end"
