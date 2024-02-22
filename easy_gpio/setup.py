from setuptools import find_packages, setup

package_name = 'easy_gpio'

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
    maintainer='Andrey Tulyakov',
    maintainer_email='mhyhre@gmail.com',
    description='Simple lgpio client node for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpio_node = easy_gpio.gpio_node:main',
        ],
    },
)
