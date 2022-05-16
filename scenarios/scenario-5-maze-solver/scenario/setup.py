from setuptools import setup

package_name = 'scenario'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/simulate.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/maze.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='b.wesselink',
    maintainer_email='b.b.a.wesselink@student.tue.nl',
    description='Package to run the scenario.',
    license='LGPL-3.0',
    tests_require=['pytest'],
    entry_points={},
)
