from setuptools import setup
import os
from glob import glob

package_name = 'uwb_simulator'

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
    maintainer='henascen',
    maintainer_email='ascencio.henan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_tf2_broadcaster = uwb_simulator.turtle_tf2_broadcaster:main',
            'tf2_listener = uwb_simulator.tf2_listener:main',
            'turtle_tf2_base_rename = uwb_simulator.turtle_tf2_base_rename:main',
            'drone_tf2_base_rename = uwb_simulator.drone_tf2_base_rename:main',
            'drone_tf2_broadcaster = uwb_simulator.drone_tf2_broadcaster:main',
            'trajectories = uwb_simulator.trajectories:main',
            'plotter = uwb_simulator.plotter:main',
        ],
    },
)
