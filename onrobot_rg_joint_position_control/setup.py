## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['onrobot_rg_joint_position_control'],
    package_dir={'': 'scripts'},
    requires=['rospy', 'onrobot_rg_control']
)

setup(**setup_args)