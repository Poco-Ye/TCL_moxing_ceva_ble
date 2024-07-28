
@echo off
set build_path="../../build"
if not exist %build_path% (
	mkdir %build_path% 
	cd %build_path%
) else (
	cd %build_path%
)


if "%1" == "" goto BuildERROR
if "%2" == "" goto BuildERROR
echo makefile generating
cmake ../tools/test_ble -G"Unix Makefiles"  -DBOARD=%1 -DSOC=%2 -DBT_SUPPORT=YES
echo start make
make
riscv-nuclei-elf-objdump.exe  -S -d   test_ble_sdk.exe > test_ble_sdk.asm

riscv-nuclei-elf-objcopy -j .func.rom -O binary "test_ble_sdk.exe"  "test_ble_sdk_rom.bin"
riscv-nuclei-elf-objcopy -R .func.rom -O binary "test_ble_sdk.exe"  "test_ble_sdk_flash.bin"
riscv-nuclei-elf-size --format=berkeley "test_ble_sdk.exe"
JFlash.exe -hide -open"../build-script/bootrom.bin,0" -merge"test_ble_sdk_rom.bin, 0x1000" -saveas"rom.bin" -jflashlog./jflog -exit
hexdump -v -e '/4 "%%08%X\n"'  "rom.bin" > "rom.hex"

start "" "../build-script/cygwin/ms_hexdump"
ping localhost -n 5 >nul


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
goto BuildEND

:BuildERROR:
echo please select board name FPGA or 1008
echo please select soc name MS1008 or  MS1008_V2
echo build error, need board and soc name

:BuildEND:
echo build end
cd %~dp0