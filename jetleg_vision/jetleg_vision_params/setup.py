from setuptools import find_packages, setup

#### Generate Parameter Library
from generate_parameter_library_py.setup_helper import generate_parameter_module

generate_parameter_module(
  "pointcloud_proc_parameters", # python module name for parameter library
  "src/pointcloud_proc_parameters.yml", # path to input yaml file
)
####

package_name = 'jetleg_vision_params'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anthony',
    maintainer_email='anthonybrown0528@protonmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
