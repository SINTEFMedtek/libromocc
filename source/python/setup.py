from setuptools import setup, find_packages
import os

package_name = "pyromocc"
package_data = {}

if os.name == 'posix':
    package_data[package_name] = ['*.so']
else:
    package_data[package_name] = ['*.pyd', '*.dll']

setup(name=package_name,
      version='0.0.2',
      packages=find_packages(exclude=['third_party', 'examples']),
      install_requires=['numpy'],
      include_package_data=True,
      package_data=package_data)
