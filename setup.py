from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['fast_simulator'],
    scripts=['scripts/spawn'],
    package_dir={'': 'src'}
)

setup(**d)
