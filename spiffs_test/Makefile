
SPIFFS_SCRIPT:= ./spiffs_upload.sh
all:
	make compile
	make upload
compile:
	arduino-cli compile -b esp32:esp32:esp32doit-devkit-v1  #compile sketch

upload:
	arduino-cli upload -p /dev/ttyUSB0 -b esp32:esp32:esp32doit-devkit-v1 #upload sketch
spiffs:
	@bash $(SPIFFS_SCRIPT)

