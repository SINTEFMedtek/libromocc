libromocc
=========

[![Build badge](https://github.com/SINTEFMedtek/libromocc/workflows/cmake_build.yml/badge.svg?branch=master&event=push)](https://github.com/SINTEFMedtek/libromocc/actions)

libromocc is a lightweight C++ library for **ro**bot **mo**delling, **c**ontrol and **c**ommunication. It
also contains Python wrappers with a corresponding PyPI distribution.

### Setup and build

```bash
git clone https://github.com/SINTEFMedtek/libromocc.git
cd libromocc
mkdir build
cd build
cmake ..
make -j8
```

### Usage

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


### Contributors

libromocc is developed at [SINTEF Digital](http://www.sintef.no), and the work has been partially funded by the Research Council of Norway under grant number 270941.
