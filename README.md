libromocc
=========

[![Build Ubuntu](https://github.com/SINTEFMedtek/libromocc/actions/workflows/build_ubuntu.yml/badge.svg)](https://github.com/SINTEFMedtek/libromocc/actions/workflows/build_ubuntu.yml)
[![Build Windows](https://github.com/SINTEFMedtek/libromocc/actions/workflows/build_windows.yml/badge.svg)](https://github.com/SINTEFMedtek/libromocc/actions/workflows/build_windows.yml)

libromocc is a lightweight C++ library for **ro**bot **mo**delling, **c**ontrol and **c**ommunication. It
also contains Python wrappers with a corresponding PyPI distribution.

### Core features ###
* **Lightweight inferface**: Simple control of robots from Universal Robots. 
* **Robot modelling**: Forward and inverse kinematics, Jacobians, etc. 
* **Support for old client interfaces**: Supports the real-time interfaces of Universal Robots, but not the new RTDE interface.
* **Python wrappers**: Python wrappers for core functionality. 

### Getting started with Python ###
[![pypi](https://badgen.net/pypi/v/pyromocc)](https://pypi.org/project/pyromocc/)

Pre-built packages are available for Linux and Windows on PyPI. Supports Ubuntu 20.04, 22.04 and Windows 10+(64-bit). 
To install the current release:

```bash
pip install pyromocc
```

Once installed, the following shows a simple sample of use:
```python
from pyromocc import Robot

# Connect to robot
robot = Robot(ip="192.168.153.131", port=30003, manipulator="UR5")
robot.connect()

# Print current operational configuration
print(robot.operational_config)

# Move 50 mm upwards
robot.z += 50

# Print current operational configuration
print(robot.operational_config)
```

### Setup and build ###

```bash
git clone https://github.com/SINTEFMedtek/libromocc.git
cd libromocc
mkdir build
cd build
cmake ..
make -j8
```

### Usage ###

If you use libromocc in your research and would like to cite the library, we suggest you cite the following conference paper:

```
@InProceedings{ostvik2019echobot,
  author    = {Andreas {\O}stvik and Lars Eirik B{\o} and Erik Smistad},
  title     = {{EchoBot}: An open-source robotic ultrasound system},
  booktitle = IPCAI,
  year      = {2019},
}
```

DISCLAIMER: libromocc is researchware and is provided "as-is". The contributors makes no other warranties, express or 
implied, and hereby disclaims all implied warranties for use and further development. The code is free to download and 
use under a BSD-2 license. See included licence for more information.

The code base is currently undergoing large changes, thus there is no guarantee that internal interfaces will be stable.


### Contributors ###

libromocc is developed at [SINTEF Digital](http://www.sintef.no), and the work has been partially funded by the Research Council of Norway under grant number 270941.
