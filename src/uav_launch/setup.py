"""webots_ros2 package setup file."""

from setuptools import setup


package_name = 'uav_launch'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/mavic_world.wbt', 'worlds/.mavic_world.wbproj',
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/mavic_webots.urdf'
]))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Sebastian Ohl',
    author_email='s.ohl@ostfalia.de',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='launch uav world',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
