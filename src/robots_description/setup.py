from setuptools import setup
import os
from glob import glob

package_name = 'robots_description'

# Add the models inside the models folder and subfolders to the share folder
models_data_files = []
directories = glob('models/*')

for directory in directories:
    files = glob(directory + '/*')
    share_files = []
    for file in files:
        is_dir = os.path.isdir(file)
        if is_dir:
            directories.append(file)
        else:
            share_files.append(file)
    directory = os.path.join('share', package_name, directory)
    models_data_files.append((directory, share_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')
        ),
        (
            os.path.join('lib', package_name),
            [package_name + '/inject_entity.py']
        ),
        *models_data_files,
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')
        )
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
        ],
    },
)
