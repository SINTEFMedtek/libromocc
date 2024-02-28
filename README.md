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
For building libromocc, you need to have CMake installed. 

```bash
git clone https://github.com/SINTEFMedtek/libromocc.git
cd libromocc
mkdir build
cd build
```

#### Windows ####
```bash
cmake .. -G "Visual Studio 16 2019" -A x64 # Change generator string to reflect your VS version
cmake --build . --config Release -j 4
```

#### Ubuntu ####

```bash
cmake ..
make -j 4
```

### Setup and build (Windows) ###

These instructions assume you have Visual Studio and CMake GUI available on your computer. 
```bash
git clone https://github.com/SINTEFMedtek/libromocc.git
cd libromocc
mkdir build
cd build
cmake-gui.exe ..
```
Update the paths to the source code and the build directory as needed:
- Source: ```<path_to>/libromocc/source```
- Build: ```<path_to>/libromocc/build```

Click Configure and select the Visual Studio version you would like to use to compile the libraries.

Update the configuration as needed e.g., build tests, examples, Python bindings, etc. Remember to click Configure to update the configuration, and update the paths of the different variables as needed (see **Building the Python bindings** underneath).

When ready, click Generate to create the Visual Studio solution files. Once done click on Open Project.

On Visual Studio, change the solution configuration to Release and build the solution (Build > Build solution).

Once compiled, using again CMake GUI, check that the installation path is correct i.e., modify ```CMAKE_INSTALL_PREFIX``` as needed.

Open the project on Visual Studio again, and compile ```ALL_BUILD``` and the ```INSTALL``` solutions. Notice that to compile the ```INSTALL``` solution, you should be running Visual Studio as admin.

Add the library to the ```PATH``` environment variable by adding the path to the bin folder in the installation directory e.g., ```%CMAKE_INSTALL_PREFIX%/romocc/libromocc/bin```

#### Building the Python bindings ####
When configuring the project using CMake, check the option ```ROMOCC_BUILD_PYTHON_BINDINGS```, and click Configure to update the configuration of the project.

Update the paths to the Python interpreter and debugger as needed. Build the project as described above.

The python wheels can be found in: ```%CMAKE_INSTALL_PREFIX%/romocc/pyromocc```

#### Using cmake from the terminal ####
Take a look at https://gnuwin32.sourceforge.net/packages/make.htm to generate the Visual Studio solution files.
```bash
git clone https://github.com/SINTEFMedtek/libromocc.git
cd libromocc
mkdir build
cd build
cmake ..
# Open the project on Visual Studio
Start-Process .\ALL_BUILD.vcxproj
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

DISCLAIMER: libromocc is researchware and is provided "as-is". The contributors make no other warranties, express or 
implied, and hereby disclaims all implied warranties for use and further development. The code is free to download and 
use under a BSD-2 licence. See included licence for more information.

The code base is currently undergoing large changes, thus there is no guarantee that internal interfaces will be stable.


### Contributors ###

libromocc is developed at [SINTEF Digital](http://www.sintef.no), and the work has been partially funded by the Research Council of Norway under grant number 270941.
