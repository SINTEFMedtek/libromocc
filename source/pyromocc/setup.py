import os
import glob

from setuptools import setup, find_packages, Extension

package_name = "pyromocc"

build_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))  # Should be dynamic
bin_path = os.path.join(build_folder, 'bin')
library_path = os.path.join(build_folder, 'lib')

if os.name == 'posix':
    libraries = glob.glob(os.path.join(library_path, "*.so"))
    package_data = {"pyromocc": ['*.so', *libraries]}
else:
    libraries = glob.glob(os.path.join(library_path, "*.lib"))
    dlls = glob.glob(os.path.join(bin_path, "*.dll"))
    pyds = glob.glob(os.path.join(library_path, "*.pyd"))
    package_data = {"pyromocc": ['*.pyd', '*.dll', '*.lib', *pyds, *libraries, *dlls]}

setup(name=package_name,
      version='0.0.6',
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
      package_data=package_data)
