pyromocc - Python wrapper for libromocc
=======================================

## Installation
To install the current release:

```bash
pip install pyromocc
```

To update, add ``--upgrade`` to the above command.

## Build from source

### Ubuntu 18.04

1. Install dependencies
```bash
sudo apt-get install python3 python3-dev cmake g++ git
```

2. Run top level CMake commands with additional flag -DROMOCC_BUILD_PYTHON_BINDINGS:bool=TRUE
```bash
mkdir build
cd build
cmake .. -DROMOCC_BUILD_PYTHON_BINDINGS:bool=TRUE
```

3. Setup for development with pip (from directory with setup.py)
```
pip install -e .
```

## Example usage
```python
from pyromocc import Robot
robot = Robot(ip="192.168.153.131", port=30003, manipulator="UR10")
robot.connect()

robot.movej([0, -1.57, -1.57, -1.57, 0, 0])
print(robot.joint_config)
```
