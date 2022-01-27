from glob import glob
import os
from setuptools import setup

package_name = 'jetleg_control'

data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
]

def glob_recursive(data_files, directory):
    files = glob(directory+'*.*')
    data_files.append((os.path.join('share', package_name, directory), files))
    subdirectories = glob(directory+'*/')
    if (subdirectories == []):
        return
    else:
        for dir in subdirectories:
            glob_recursive(data_files, dir)

data_directories = ['launch']

for directory in data_directories:
    glob_recursive(data_files, directory)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jasonx',
    maintainer_email='59701038+JChunX@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        		'jetleg_teleop_key = jetleg_control.jetleg_teleop_key:main',
        ],
    },
)
