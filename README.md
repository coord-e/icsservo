# icsservo

![Travis (.org)](https://img.shields.io/travis/coord-e/icsservo.py.svg)
![PyPI](https://img.shields.io/pypi/v/icsservo.svg)
![PyPI - License](https://img.shields.io/pypi/l/icsservo.svg)
![C++](https://img.shields.io/badge/C%2B%2B-14-orange.svg)

ICSServo: ICS serial servo driver library

## Install

```shell
pip install icsservo
```

## Serial port setup

### Raspberry Pi 3 B

```shell
# Disable Bluetooth and Enable PL011 uart.
# MiniUART cannot be used because it does not support parity.
# https://www.raspberrypi.org/documentation/configuration/uart.md
echo 'dtoverlay=pi3-miniuart-bt' | sudo tee -a /boot/config.txt

# Reboot to apply changes
sudo reboot

# Configure serial port with stty(1)
stty -F /dev/serial0 115200 raw parenb
```

## Usage

```python
from icsservo import IOProvider

with IOProvider(device="/dev/serial0", en_idx=23) as io:
  servo = io.servo(1) # Servo with ID 1
  servo.set_position(3.14 / 2)
```
