## esp32 esp-idf
https://www.youtube.com/watch?v=Jt6ZDct4bZk
```c
python -V
which pythoh3
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 l
sudo apt install -y python3-pip
pip3 -V
which pip3
sudo update-alternatives --install /usr/bin/pip pip /usr/bin/pip3 l
sudo apt install -y git wget flex bison gperf python-setuptools cmake ninja-build ccache libffi-dev libssl-dev dfu-util
mkdir esp && cd esp
git clone --recursive https://github.com/espressif/esp-idf.git
mkdir esp-idf && cd esp-idf
. ./install.sh
pip install --upgrad pip  # if have warning after install.sh
. ./export.sh
printenv    # Check enviroment
  IDF_TOOLS_EXPORT_CMD......
  IDF_TOOLS_INSTALL_CMD......
nano ~/.profile   # go down below and insert 
. $HOME/esp/esp-idf/export.sh
nano ~/.bashrc     # go down below and insert
. $HOME/esp/esp-idf/export.sh
------
sudo usermod -a -G dialout,tty $USER
connect to ESP32
ls /dev/tty*

cp -r ledc_basic/ ~/Desktop/

cd Desktop
cp -r $IDF_PATH/examples/get-started/hello_world .
cp -r $IDF_PATH/examples/get-started/blink .
cp -r $IDF_PATH/examples/peripherals/gpio/generic_gpio .
gpio lcd ledc mcpwm i2c
cd blink
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py monitor
```
## ledc => led control
* https://medium.com/jungletronics/esp32-idf-ledc-get-started-f973c4b7e41e
* https://github.com/espressif/esp-idf/tree/master/examples/peripherals
