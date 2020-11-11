#!/bin/bash

SERIALBORD=/dev/tty.usbserial-AI02RM14 #/dev/tty.usbserial-AI02RM14
PM=disabled #disabled,enabled
PARTITION_SCHEME=default #default
F_CPU=20 #10,13.20,26,40.80,160,240
FLASH_MODE=qio #dio,qio,dout,qout
F_FLASH=80 #40,80
FLASH_SIZE=4M #2M,4M,8M,16M
UPLOAD_SPEED=921600 #115200,230400,460800,921600
DEBUG_LEVEL=verbose #none,error,warn,info,debug,verbose
SKECH_PATH=../BNO055_compass/

arduino-cli compile -v --fqbn esp32:esp32:esp32:PSRAM=$PM,PartitionScheme=$PARTITION_SCHEME,CPUFreq=$F_CPU,FlashMode=$FLASH_MODE,FlashFreq=$F_FLASH,FlashSize=$FLASH_SIZE,UploadSpeed=$UPLOAD_SPEED,DebugLevel=$DEBUG_LEVEL $SKECH_PATH
arduino-cli upload -v -p $SERIALBORD --fqbn esp32:esp32:esp32:PSRAM=$PM,PartitionScheme=$PARTITION_SCHEME,CPUFreq=$F_CPU,FlashMode=$FLASH_MODE,FlashFreq=$F_FLASH,FlashSize=$FLASH_SIZE,UploadSpeed=$UPLOAD_SPEED,DebugLevel=$DEBUG_LEVEL $SKECH_PATH
