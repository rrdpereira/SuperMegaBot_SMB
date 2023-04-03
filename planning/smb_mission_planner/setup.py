from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['smb_mission_planner'],
    scripts=['scripts'],
    package_dir={'': 'src'}
)

setup(**d)