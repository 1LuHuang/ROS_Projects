from setuptools import find_packages, setup

package_name = 'simple_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/simple_car.launch.py']),
    ('share/' + package_name + '/worlds', ['worlds/simple_world.sdf']),
    ('share/' + package_name + '/models', ['models/simple_model.sdf']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yiluhuang',
    maintainer_email='yiluhuangs@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'figure8_path_node = simple_car.traj_sim.figure8_path_node:main',
            'pure_pursuit_node = simple_car.traj_sim.pure_pursuit_node:main',
            'actual_path_node = simple_car.traj_sim.actual_path_node:main',
            'error_logger_node = simple_car.traj_sim.error_logger_node:main',
            'pid_tracker_node = simple_car.traj_sim.pid_tracker_node:main',
        ],
    },
)
