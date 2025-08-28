from setuptools import setup

package_name = 'pid_tuner'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@ros.com',
    description='A simple tool to send timed Twist commands for PID tuning.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tune = pid_tuner.tuner_node:main',
            'tune_kb = pid_tuner.tuner_keyboard:main',
        ],
    },
)
