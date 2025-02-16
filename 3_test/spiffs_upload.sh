#!/bin/bash

# Paths
MK_SPIFFS="/home/nicko/.arduino15/packages/esp32/tools/mkspiffs/0.2.3/mkspiffs"
ESPTOOL=/usr/bin/esptool.py
DATA_DIR=$PWD/data
#BIN_FILE="/tmp/arduino_build_985055/spiffs_test.spiffs.bin"
BIN_FILE=spiffs.bin
PORT="/dev/ttyUSB0"
BAUD=921600
FLASH_ADDR=2686976

# SPIFFS parameters (match your ESP32 settings)
PAGE_SIZE=256
BLOCK_SIZE=4096
FS_SIZE=1441792

echo "Creating SPIFFS binary..."
$MK_SPIFFS -c $DATA_DIR -p $PAGE_SIZE -b $BLOCK_SIZE -s $FS_SIZE $BIN_FILE

if [ $? -eq 0 ]; then
    echo "SPIFFS binary created successfully!"
else
    echo "Failed to create SPIFFS binary."
    exit 1
fi

echo "Flashing SPIFFS to ESP32..."
/usr/bin/python $ESPTOOL \
  --chip esp32 \
  --baud $BAUD \
  --port $PORT \
  --before default_reset \
  --after hard_reset \
  write_flash -z \
  --flash_mode dio \
  --flash_freq 80m \
  --flash_size detect \
  $FLASH_ADDR $BIN_FILE

if [ $? -eq 0 ]; then
    echo "SPIFFS flashed successfully!"
else
    echo "Failed to flash SPIFFS."
    exit 1
fi

