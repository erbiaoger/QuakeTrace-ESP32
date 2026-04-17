pio run -t clean
pio run -t upload --upload-port /dev/cu.usbserial-0001
pio device monitor -b 115200 --port /dev/cu.usbserial-0001
