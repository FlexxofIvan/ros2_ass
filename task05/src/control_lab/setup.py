from setuptools import setup
from glob import glob
setup(name='control_lab',version='2026.1.0',packages=['control_lab'],
 data_files=[('share/ament_index/resource_index/packages',['resource/control_lab']),
 ('share/control_lab',['package.xml']),('share/control_lab/launch',glob('launch/*.py')),
 ('share/control_lab/config',glob('config/*'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='SHAD ROS 2 course',
 maintainer_email='course@example.invalid',description='Control fundamentals on an original humanoid',license='MIT',
 entry_points={'console_scripts':['lab = control_lab.cli:main','pid_lab = control_lab.pid:main']})
