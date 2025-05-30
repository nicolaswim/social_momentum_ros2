import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_social_nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wim',
    maintainer_email='wim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'social_planner_node = my_social_nav_pkg.social_planner_node:main',
            'fake_human_publisher = my_social_nav_pkg.fake_human_publisher:main', #<-- Verify this line EXACTLY
        ],
    },
)
