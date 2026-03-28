from setuptools import setup

package_name = 'mars_platform'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/platform_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aditya',
    maintainer_email='aditya@todo.todo',
    description='Moving platform controller for MARS project',
    license='MIT',
    entry_points={
        'console_scripts': [
            'platform_mover = mars_platform.platform_mover:main',
        ],
    },
)
