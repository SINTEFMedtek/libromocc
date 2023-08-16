import os
import glob

from setuptools import setup, find_packages

package_name = "pyromocc"
package_data = {}

library_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../build/lib'))

if os.name == 'posix':
    libraries = glob.glob(os.path.join(library_path, "*.so"))
    package_data = {"pyromocc": ['*.so', *libraries]}
else:
    libraries = glob.glob(os.path.join(library_path, "*.lib"))
    package_data[package_name] = ['*.pyd', '*.dll', '*.lib']

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
          'Programming Language :: C++',
          'Programming Language :: Python :: 3.7',
          'Programming Language :: Python :: 3.8',
          'Programming Language :: Python :: 3.9',
      ],
      python_requires='>=3.7',
      package_data=package_data)
