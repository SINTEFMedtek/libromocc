import os
import glob
import platform
import sys

from setuptools import setup, find_packages

package_name = "pyromocc"
version = "0.0.6"

build_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), 'build'))
bin_path = os.path.join(build_folder, 'bin')
library_path = os.path.join(build_folder, 'lib')

if os.name == 'posix':
    libraries = glob.glob(os.path.join(library_path, "*.so"))
    package_data = {"pyromocc": ['*.so', *libraries]}
    platform_name = "none"
elif os.name == 'nt':
    libraries = glob.glob(os.path.join(library_path, "*.lib"))
    dlls = glob.glob(os.path.join(bin_path, "*.dll"))
    pyds = glob.glob(os.path.join(library_path, "*.pyd"))
    platform_name = f"win_{platform.machine()}"
    package_data = {"pyromocc": ['*.pyd', '*.dll', '*.lib', *pyds, *libraries, *dlls]}
else:
    raise NotImplementedError(f"Platform {os.name} currently not supported")

python_version = f"cp{sys.version_info.major}{sys.version_info.minor}"

setup(name=package_name,
      version=version,
      author="Andreas Oestvik",
      packages=find_packages(exclude=['examples']),
      install_requires=['numpy'],
      setup_requires=['wheel'],
      extras_require={'examples': ["matplotlib"]},
      include_package_data=True,
      classifiers=[
          'Operating System :: POSIX :: Linux',
          'Operating System :: Microsoft :: Windows',
          'Programming Language :: C++',
          'Programming Language :: Python :: 3.7',
          'Programming Language :: Python :: 3.8',
          'Programming Language :: Python :: 3.9',
      ],
      python_requires='>=3.7',
      script_args=["bdist_wheel", "--python-tag", python_version, "--plat-name", platform_name],
      package_data=package_data)
