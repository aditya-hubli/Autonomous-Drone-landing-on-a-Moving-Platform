from setuptools import setup

package_name = 'mars_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aditya',
    maintainer_email='aditya@todo.todo',
    description='ArUco marker perception for MARS project',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aruco_detector = mars_perception.aruco_detector:main',
        ],
    },
)
