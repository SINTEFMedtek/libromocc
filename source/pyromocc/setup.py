from setuptools import setup, find_packages
import os

package_name = "pyromocc"
package_data = {}

if os.name == 'posix':
    package_data[package_name] = ['*.so']
else:
    package_data[package_name] = ['*.pyd', '*.dll']

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
          'Programming Language :: Python :: 3.6',
          'Programming Language :: Python :: 3.7',
          'Programming Language :: Python :: 3.8',
      ],
      python_requires='>=3.6',
      package_data=package_data)
