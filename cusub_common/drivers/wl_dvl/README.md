# dvl-python Python library to serial interface with Water Linked DVLs

> :warning: This library is in development and is not yet production ready

Python code to serial interface with Water Linked DVLs.

The library makes setting up a serial connection with the DVL simpler. Handles the parsing, checks validity and returns a Dictionary.

## Resources

- [Water Linked web site](https://waterlinked.com/dvl/)
- [DVL A50 documentation](https://waterlinked.github.io/dvl/dvl-a50/)
- [DVL protocol specification](https://waterlinked.github.io/dvl/dvl-protocol/)
- [Repository](https://github.com/waterlinked/dvl-python)

## Requirements

- Python 3 or 2
- crcmod
- pyserial

```bash
pip install crcmod pyserial
```

You might also need additional permission to access the port on the system you're running the script

## Supported DVLs

- Water Linked DVL A50

## Setup

Download or clone the repo.

```bash
git clone https://xolgit.orbitlogic.com/minau/waterlinked-dvl-python.git
```

Make sure you're in the setup folder with the setup.py file. Install the module (Note the period at the end of the command. The -e will let you edit the module as you wish.):

```bash
cd waterlinked-dvl-python/setup
pip install -e .
```

## Minau Quick Start

Figure out what serial port the dvl is going to. Replace `ttyS0` with this serial port in following instructions. We are going to switch the serial port on boot. To do so add the following to the subs /etc/rc.local file:

```bash
#!/bin/bash
sudo mv /dev/ttyS0 /dev/dvl
sudo chmod 666 /dev/dvl
exit 0
```

If your sub does not have a `/etc/rc.local` file. Just make one and mark it as executable.

Now you can simpliy do:

```bash
rosrun ros_dvl real_dvl.py
```

And that will pubilish the dvl data to the wl_dvl topic.

Add this to bashrc

```bash
export PYTHONPATH=$HOME/dirWithScripts/:$PYTHONPATH
```

## Quick start

Connecting to a dvl and reading data:

```py
$ python

>>>  from wldvl import WlDVL
>>>  dvl = WlDVL("/dev/dvl")
>>>  dvl.read()
{'fom': 0.002, 'time': 40.57, 'vy': 0.004, 'vz': -0.002, 'vx': -0.003, 'valid': True, 'altitude': 0.14}
```

## Usage

The `WlDVL` class provides an easy interface to receive data with a Water Linked DVL.

A `WlDVL` object is initialized with the serial device port:

```py
from wldvl import WlDVL
dvl = WlDVL("/dev/dvl")
```

To retrieve data as a dictionary:

```py
dvl.read()
```

This should give you a dictionary formated as follows:

```python
{
    'time': 40.75,
    'vx': 0.001,
    'vy': 0.004,
    'vz': -0.001,
    'fom': 0.002,
    'altitude': 0.13,
    'valid': True
}
```

## Examples

Examples showing how to use the API is available in the [examples/](examples/) folder.
