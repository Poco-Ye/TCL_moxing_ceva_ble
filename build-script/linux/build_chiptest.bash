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
cmake ../components/platform/hal/ms1008verify -G"Unix Makefiles"   -DBOARD=$board_name -DSOC=$soc_name
echo "start make"
make
riscv-nuclei-elf-objdump  -S -d   test_ms1008 > test_ms1008.asm
riscv-nuclei-elf-objcopy -O binary "test_ms1008"  "test_ms1008.bin"
riscv-nuclei-elf-size --format=berkeley "test_ms1008"

echo "build end"
