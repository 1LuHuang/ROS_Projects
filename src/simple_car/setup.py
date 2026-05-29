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
    ('share/' + package_name + '/worlds', ['worlds/world_withObs.sdf']),
    ('share/' + package_name + '/worlds', ['worlds/world_dyn_Obs.sdf']),
    ('share/' + package_name + '/launch', ['launch/simple_car_localPlan.launch.py']),
    ('share/' + package_name + '/launch', ['launch/simple_car_dynamicPlan.launch.py']),
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
            "a_star_node = simple_car_localplan.traj_sim.path_publisher_node:main",
            'pure_pursuit_node_AStar = simple_car_localplan.traj_sim.pure_pursuit_node:main',
            "dynamic_obstacle_node = simple_car_dynamicPlan.traj_sim.dynamic_obstacle_node:main",
            "dynamic_detector_node = simple_car_dynamicPlan.traj_sim.dynamic_detector_node:main",
            "pure_pursuit_node_AStar_DynObs = simple_car_dynamicPlan.traj_sim.pure_pursuit_node:main",
        ],
    },
)
