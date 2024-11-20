from setuptools import find_packages, setup

package_name = 'px4_motor_error'

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
    maintainer='shenlong',
    maintainer_email='149168521+dev-shenlong@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_logger = px4_motor_error.sensor_logger:main',
            'offboard_controller = px4_motor_error.offboard_controller:main',
            'fail_motor = px4_motor_error.fail_motor:main',
            'motor_error_detector = px4_motor_error.motor_error_detector:main'
        ],
    },
)
