from setuptools import find_packages, setup

package_name = 'ro45_portalrobot_controller'

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
    maintainer='sebi',
    maintainer_email='ratzeray@gmail.com',
    description='controls the robot by sending and receiving messages',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['acc_control = ro45_portalrobot_controller.user_input:main',
        ],
    },
)
