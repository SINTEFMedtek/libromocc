import os
import glob
import platform
import shutil

from setuptools.command.install import install
from setuptools.dist import Distribution
from setuptools import setup, find_packages

package_name = "pyromocc"
version = "0.0.6"

build_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
bin_path = os.path.join(build_folder, 'bin')
library_path = os.path.join(build_folder, 'lib')

if platform.system() == "Linux":
    libraries = glob.glob(os.path.join(library_path, "*.so"))
    package_data = {"pyromocc": ['*.so', *libraries]}
elif platform.system() == "Windows":
    libraries = glob.glob(os.path.join(library_path, "*.lib"))
    dlls = glob.glob(os.path.join(bin_path, "*.dll"))
    pyds = glob.glob(os.path.join(library_path, "*.pyd"))
    package_data = {"pyromocc": [*libraries, *dlls, *pyds]}
else:
    raise NotImplementedError(f"Platform {platform.system()} currently not supported")


class CustomInstallCommand(install):
    def run(self):
        # run the original install command
        install.run(self)

        # copy the .pyd and .dll files to site-packages
        module_dir = self.install_lib

        if platform.system() == "Windows":
            for deps in [*dlls, *pyds, *libraries]:
                shutil.copy(deps, os.path.join(module_dir, "pyromocc"))


class BinaryDistribution(Distribution):
    """Distribution which always forces a binary package with platform name"""
    def has_ext_modules(self):
        return True


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
      package_data=package_data,
      cmdclass={'install': CustomInstallCommand},
      distclass=BinaryDistribution)
