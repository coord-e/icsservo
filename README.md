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

## Usage

```python
from icsservo import IOProvider

with IOProvider(device="/dev/ttyS0", en_idx=23) as io:
  servo = io.servo(1) # Servo with ID 1
  servo.set_position(3.14 / 2)
```
