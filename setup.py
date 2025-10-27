from setuptools import find_packages, setup

package_name = 'servo_helpers'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Merlin Ortner',
    maintainer_email='ortnermerlin@gmail.com',
    description='A ROS 2 package that subscribes to a command topic and exposes the received commands as a service interface, enabling network-accessible control of servo motors.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'servo_relay = servo_helpers.servo_relay_node:main',
    ],
},

)
