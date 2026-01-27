from setuptools import find_packages, setup

package_name = 'pycap_franka'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'pinocchio', 'rclpy', 'sensor_msgs'],
    zip_safe=True,
    maintainer='askuric',
    maintainer_email='antun.skuric@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'franka_capacity = pycap_franka.franka_capacity:main'
        ],
    },
)
