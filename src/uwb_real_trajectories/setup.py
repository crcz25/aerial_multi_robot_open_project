from setuptools import setup
import os
from glob import glob

package_name = 'uwb_real_trajectories'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='crcz',
    maintainer_email='crcueto25@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tbot_trajectories = uwb_real_trajectories.turtlebot_trajectories:main',
            'tello_trajectories = uwb_real_trajectories.tello_trajectories:main',
            'tbot_trajectories_opti = uwb_real_trajectories.turtlebot_trajectories_opti:main',
            'save_data = uwb_real_trajectories.save_data:main',
        ],
    },
)
