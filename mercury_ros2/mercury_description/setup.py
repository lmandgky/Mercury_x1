from setuptools import setup
from setuptools import find_packages
import os

from glob import glob

package_name = 'mercury_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf'+'/mercury_a1', glob("urdf/mercury_a1/*")),
        ('share/' + package_name + '/urdf'+'/mercury_b1', glob("urdf/mercury_b1/*")),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='u2',
    maintainer_email='u2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
