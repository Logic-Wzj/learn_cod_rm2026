from setuptools import find_packages, setup

package_name = 'cod_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/loopback_sim_launch.py',
                                               'launch/fzsd_sim_launch.py']),
        ('share/' + package_name + '/config', ['config/fzsd_sim_params.yaml',
                                               'config/gt_sim_params.yaml']),
    ],
    install_requires=['setuptools'],
    include_package_data=True,
    zip_safe=True,
    maintainer='COD',
    description='Loopback 仿真',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fake_odom = cod_sim.fake_odom:main',
            'gt_odom = cod_sim.gt_odom:main',
        ],
    },
)
