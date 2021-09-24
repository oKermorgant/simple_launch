from setuptools import find_packages
from setuptools import setup
import os
from glob import glob

package_name = 'simple_launch'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages('src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
    (os.path.join('share', package_name), ['package.xml']),
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    # Include examples.
    (os.path.join('share', package_name, 'example'), glob('example/[!_]*'))
    ],
    install_requires=['setuptools'],
    author='Olivier Kermorgant',
    author_email='olivier.kermorgant@ec-nantes.fr',
    maintainer='Olivier Kermorgant',
    maintainer_email='olivier.kermorgant@ec-nantes.fr',
    url='https://github.com/oKermorgant/simple_launch',
    download_url='https://github.com/oKermorgant/simple_launch',
    keywords=['ROS 2', 'launch'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Python helpers for the ROS 2 launch system',
    license='MIT',
        # scripts here.
    entry_points={
        'console_scripts': [
            'frame_prefix_gazebo = simple_launch.frame_prefix_gazebo:main'
        ],
    },
)
