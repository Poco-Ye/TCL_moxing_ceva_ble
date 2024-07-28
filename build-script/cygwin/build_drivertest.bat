
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
cmake ../tools/test_drivers -G"Unix Makefiles"  -DBOARD=%1 -DSOC=%2
echo start make
make
riscv-nuclei-elf-objdump.exe  -S -d   test_driver_sdk.exe > test_driver_sdk.asm
riscv-nuclei-elf-objcopy -O binary "test_driver_sdk.exe"  "test_driver_sdk.bin"
riscv-nuclei-elf-size --format=berkeley "test_driver_sdk.exe"
goto BuildEND

:BuildERROR:
echo please select board name FPGA or 1008
echo please select soc name MS1008 or  MS1008_V2
echo build error, need board and soc name

:BuildEND:
echo build end
cd %~dp0