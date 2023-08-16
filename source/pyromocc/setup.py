import os
import glob

from setuptools import setup, find_packages, Extension

package_name = "pyromocc"
package_data = {}

build_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../build'))  # Should be dynamic
library_path = os.path.join(build_folder, 'lib')

if os.name == 'posix':
    libraries = glob.glob(os.path.join(library_path, "*.so"))
    package_data = {"pyromocc": ['*.so', *libraries]}
else:
    libraries = glob.glob(os.path.join(library_path, "*.lib"))
    package_data[package_name] = ['*.pyd', '*.dll', '*.lib']

extension = Extension(
    'pyromocc',
    sources=['source/pyromocc.cpp'],
    libraries=libraries,
    library_dirs=[library_path],
    include_dirs=[os.path.join(build_folder, 'third-party/pybind11/include'),
                  os.path.join(build_folder, 'include'),
                  os.path.join(build_folder, 'include/eigen3'),
                  os.path.join(build_folder, 'include/kdl'),
                  os.path.join(build_folder, '../source'),
                  os.path.join(build_folder, '.')
                  ],
    extra_compile_args=[],
    extra_link_args=['-Wl,-rpath,$ORIGIN'],
)

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
      ext_modules=[extension],
      package_data=package_data)
