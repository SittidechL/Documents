https://randomnerdtutorials.com/flashing-micropython-firmware-esptool-py-esp32-esp8266/
https://www.artronshop.co.th/
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html#get-started-windows-tools-installer
https://dl.espressif.com/dl/esp-idf/?idf=4.4
==>install ESP-IDF for flash

https://www.youtube.com/watch?v=-yNyzuNyyqA
https://github.com/freedomwebtech/cp210driver.git

https://www.youtube.com/watch?v=4UUs21EgIK4
https://github.com/spacehuhn
==>C2102

https://www.youtube.com/watch?v=ApOwrmX0TB0
==>Install micropython

========
Down load --> ESP-IDF 
Down load --> CP210XX (USB to Serial)
Down load --> Putty

----------------------------------------------------------------------------
https://stackoverflow.com/questions/54892118/esptool-py-not-recognized-as-internal-or-external-command-operable-program-or-b
-----------------------Erase flash-----------------------------------------
C:\Windows\System32>esptool.py --chip esp32 -p COM5 erase_flash
esptool.py v3.2
Serial port COM5
Connecting.........
Chip is ESP32-D0WDQ6 (revision 1)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz
MAC: ec:94:cb:6f:17:c8
Uploading stub...
Running stub...
Stub running...
Erasing flash (this may take a while)...
Chip erase completed successfully in 14.2s
Hard resetting via RTS pin...
--------------------------Write flash---------------------------------------
C:\Windows\System32>esptool.py --chip esp32 -p COM5 write_flash -z 0x1000 D:\Pex\Download\esp32-20220117-v1.18.bin
esptool.py v3.2
Serial port COM5
Connecting............
Chip is ESP32-D0WDQ6 (revision 1)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz
MAC: ec:94:cb:6f:17:c8
Uploading stub...
Running stub...
Stub running...
Configuring flash size...
Flash will be erased from 0x00001000 to 0x0017cfff...
Compressed 1555136 bytes to 1022998...
Wrote 1555136 bytes (1022998 compressed) at 0x00001000 in 91.4 seconds (effective 136.2 kbit/s)...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
-----------------------------------------------------------------

---------------------VS Code + Ampy --------------------------
PS D:\Pex\Tutorial> pip install adafruit-ampy

PS D:\Pex\Tutorial> ampy --port COM5 run test.py
--------------------------------------------------------------

