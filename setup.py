from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'listener_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdulrahman S. Al-Batati',
    maintainer_email='asmalbatati@hotmail.com',
    description='This pkg is made to listen for ROS topics and record the message in a CSV file.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_listener = listener_recorder.trajectory_listener:main',
            'target_recorder = listener_recorder.target_pose_recorder:main'
        ],
    },
)
