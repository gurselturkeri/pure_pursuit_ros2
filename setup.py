from setuptools import setup

package_name = 'pure_pursuit_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pure_pursuit.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='gurselturkeri@hotmail.com',
    description='Pure Pursuit ROS 2 controller with configurable parameters and topics',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit_ros2.pure_pursuit_node:main',
        ],
    },
)
