from setuptools import find_packages, setup

package_name = 'fake_laser_worlds'

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
    maintainer='ubuntu-22',
    maintainer_email='bilal.mscss21@iba-suk.edu.pk',
    description='Simulated laser scan environments for testing SLAM and navigation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_laser_boxes = fake_laser_worlds.fake_laser_boxes:main',
            'fake_laser_corridor = fake_laser_worlds.fake_laser_corridor:main',
            'fake_laser_ellipse = fake_laser_worlds.fake_laser_ellipse:main',
            'fake_laser_furniture = fake_laser_worlds.fake_laser_furniture:main',
            'fake_laser_maze = fake_laser_worlds.fake_laser_maze:main',
            'fake_laser_maze2 = fake_laser_worlds.fake_laser_maze2:main',
            'fake_laser_room = fake_laser_worlds.fake_laser_room:main',
            'fake_laser_sine = fake_laser_worlds.fake_laser_sine:main',
            'fake_laser_ushape = fake_laser_worlds.fake_laser_ushape:main',
            'fake_laser_wall = fake_laser_worlds.fake_laser_wall:main',
        ],
    },
)
