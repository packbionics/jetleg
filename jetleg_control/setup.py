import os
from glob import glob

from setuptools import setup
from generate_parameter_library_py.setup_helper import generate_parameter_module


package_name = 'jetleg_control'
submodules = ['jetleg_control.script', 'jetleg_control.controllers', 'jetleg_control.model']

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


data_directories = ['launch', 'config', 'docs', 'src']

for directory in data_directories:
    glob_recursive(data_files, directory)

# Generate ROS parameters from YAML description
generate_parameter_module(
    "classifier_parameters",
    "src/classifier_parameters.yaml",
)


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name] + submodules,
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jasonx',
    maintainer_email='59701038+JChunX@users.noreply.github.com',
    description='Control implementation for JetLeg',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'jetleg_teleop_key = jetleg_control.script.jetleg_teleop_key:main',
                'jetleg_gait_generator = jetleg_control.script.jetleg_gait_generator:main',
                'forwarder = jetleg_control.script.forwarder:main',
                'impedance_controller = jetleg_control.controller.impedance_controller:main',
                'simple_reflex_node = jetleg_control.simple_reflex_node:main',
                'imu_pose_estimator = jetleg_control.script.estimate_imu_pose:main'
        ],
    },
)
