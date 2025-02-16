all:
	make compile
	make upload
#	make spiffs_upload
compile:
	arduino-cli compile -b esp32:esp32:esp32doit-devkit-v1  #compile sketch

upload:
	arduino-cli upload -p /dev/ttyUSB0 -b esp32:esp32:esp32doit-devkit-v1 #upload sketch

spiffs_upload:
	mkspiffs -c data -b 8192 -p 256 -s 0x160000 spiffs.bin   #create .bin from data folder
	esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 write_flash 0x290000 spiffs.bin #flash SPIFFS
