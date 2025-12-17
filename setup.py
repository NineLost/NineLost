from setuptools import setup

package_name = 'my_robot_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dual_lidar_with_merge.launch.py']),
        ('share/' + package_name + '/config', ['config/dual_lidar_merge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lota',
    maintainer_email='lota@example.com',
    description='My robot config',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
