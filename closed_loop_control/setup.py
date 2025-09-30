from setuptools import setup

package_name = 'closed_loop_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', ['config/motor_config.yaml']),
        (f'share/{package_name}/launch', ['launch/start_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Phavit',
    maintainer_email='you@example.com',
    description='Stepper closed-loop control (Python only)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'encoder_node = closed_loop_control.encoder_node:main',
            'controller_node = closed_loop_control.controller_node:main',
        ],
    },
)