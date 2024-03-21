import os
import glob
import platform
import shutil

from os import popen
from setuptools.command.install import install as _install
from setuptools.dist import Distribution
from setuptools import setup, find_packages

package_name = "pyromocc"
version = "0.0.8"
cmdclass = {}

build_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
bin_path = os.path.join(build_folder, 'bin')
library_path = os.path.join(build_folder, 'lib')

if platform.system() == "Linux":
    libraries = glob.glob(os.path.join(library_path, "*.so"))
    package_data = {"pyromocc/pyromocc": [*libraries], "pyromocc": ["*.so"]}
elif platform.system() == "Windows":
    libraries = glob.glob(os.path.join(library_path, "*.lib"))
    dlls = glob.glob(os.path.join(bin_path, "*.dll"))
    pyds = glob.glob(os.path.join(library_path, "*.pyd"))
    package_data = {"pyromocc": [*libraries, *dlls, *pyds]}
else:
    raise NotImplementedError(f"Platform {platform.system()} currently not supported")


try:
    from wheel.bdist_wheel import bdist_wheel as _bdist_wheel

    class bdist_wheel(_bdist_wheel):

        def finalize_options(self):
            _bdist_wheel.finalize_options(self)
            if platform.system() == "Windows":
                self.root_is_pure = False

        def get_tag(self):
            _, _, plat = _bdist_wheel.get_tag(self)
            if platform.system() == "Linux":
                glibc_major, glibc_minor = popen("ldd --version | head -1").read().split()[-1].split(".")

                if glibc_major == "2" and glibc_minor == "17":
                    plat = "manylinux_2_17_x86_64.manylinux2014_x86_64"
                else:  # For manylinux2014 and above, no alias is required
                    plat = f"manylinux_{glibc_major}_{glibc_minor}_x86_64"

            return _bdist_wheel.get_tag(self)[:2] + (plat,)

    cmdclass['bdist_wheel'] = bdist_wheel
except ImportError as error:
    print("Error importing dependencies:")
    print(error)
    bdist_wheel = None


class install(_install):
    def finalize_options(self):
        _install.finalize_options(self)

        if platform.system() == "Linux":
            self.install_libbase = self.install_platlib
            self.install_lib = self.install_platlib
        else:
            module_dir = self.install_lib
            pyromocc_dir = os.path.join(module_dir, "pyromocc")

            if not os.path.exists(pyromocc_dir):
                os.makedirs(pyromocc_dir)

            for deps in [*libraries]:
                shutil.copy(deps, pyromocc_dir)

            if platform.system() == "Windows":
                for deps in [*dlls, *pyds]:
                    shutil.copy(deps, pyromocc_dir)


cmdclass['install'] = install


class BinaryDistribution(Distribution):
    """Distribution which always forces a binary package with platform name"""
    def has_ext_modules(self):
        return True


setup(name=package_name,
      version=version,
      author="Andreas Oestvik",
      packages=find_packages(exclude=['examples']),
      install_requires=['wheel', 'numpy'],
      setup_requires=['wheel'],
      extras_require={'examples': ["matplotlib"]},
      include_package_data=True,
      classifiers=[
          'Operating System :: POSIX :: Linux',
          'Operating System :: Microsoft :: Windows',
          'Programming Language :: C++',
          'Programming Language :: Python :: 3.8',
          'Programming Language :: Python :: 3.9',
          'Programming Language :: Python :: 3.10',
      ],
      python_requires='>=3.8',
      package_dir={'pyromocc': 'pyromocc'},
      package_data=package_data,
      cmdclass=cmdclass,
      distclass=BinaryDistribution)
