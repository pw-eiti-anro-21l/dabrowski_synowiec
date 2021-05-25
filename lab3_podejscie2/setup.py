import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'lab3_podejscie2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('lab3_podejscie2/*')),
        (os.path.join('share', package_name), glob('*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pdabrow1',
    maintainer_email='piotr.dabrowski11.stud@pw.edu.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'nonkdl = lab3_podejscie2.nonkdl_dkin:main',
        	'kdl = lab3_podejscie2.kdl_dkin:main'
        ],
    },
)
