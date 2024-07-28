

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
cmake ../components/platform/hal/ms1008verify -G"Unix Makefiles"   -DBOARD=%1 -DSOC=%2
echo start make
make
riscv-nuclei-elf-objdump.exe  -S -d   test_ms1008.exe > test_ms1008.asm
riscv-nuclei-elf-objcopy -O binary "test_ms1008.exe"  "test_ms1008.bin"
riscv-nuclei-elf-size --format=berkeley "test_ms1008.exe"
goto BuildEND

:BuildERROR:
echo please select board name FPGA or 1008
echo please select soc name MS1008 or  MS1008_V2
echo build error, need board and soc name

:BuildEND:
echo build end
cd %~dp0