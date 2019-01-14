#!/bin/sh
OPENOCD_PATH=/home/thomas/Ac6/SystemWorkbench/plugins/fr.ac6.mcu.externaltools.openocd.linux64_1.20.0.201809060829/tools/openocd
OPENOCD_SCRIPTS_PATH=/home/thomas/Ac6/SystemWorkbench/plugins/fr.ac6.mcu.debug_2.2.0.201807130628/resources/openocd/st_scripts
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$OPENOCD_PATH/lib

$OPENOCD_PATH/bin/openocd -f "KugleFirmware/KugleFirmware Debug.cfg" -s "$OPENOCD_SCRIPTS_PATH" -c "init; reset halt; flash info 0; flash erase_sector 0 0 last; flash erase_check 0; exit"
# See https://github.com/trezor/trezor-core/blob/master/Makefile#L177

$OPENOCD_PATH/bin/openocd -f "KugleFirmware/KugleFirmware Debug.cfg" -s "$OPENOCD_SCRIPTS_PATH" -c "init; reset halt; flash info 1; flash erase_sector 1 0 last; flash erase_check 1; exit"
# See https://github.com/trezor/trezor-core/blob/master/Makefile#L177

## Flashing can also be done from commandline by adding commands:
# -c "init" -c "targets" -c "reset halt" -c "load_image firmware.elf" -c "reset halt" -c "verify_image firmware.elf" -c "reset run" -c "resume"

# Flash
#$OPENOCD_PATH/bin/openocd -f "KugleFirmware/KugleFirmware Debug.cfg" -s "$OPENOCD_SCRIPTS_PATH" -c "init" -c "reset init" -c "halt" -c "flash write_image erase KugleFirmware/Debug/KugleFirmware.bin 0x08000000" -c "shutdown"

# Reset
#$OPENOCD_PATH/bin/openocd -f "KugleFirmware/KugleFirmware Debug.cfg" -s "$OPENOCD_SCRIPTS_PATH" -c "init" -c "targets" -c "reset halt" -c "reset run" -c "exit"
