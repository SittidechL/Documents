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

ls /dev/tty*



cp -r $IDF_PATH/example/get-started/blink .

```

