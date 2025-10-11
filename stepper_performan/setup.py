from setuptools import find_packages, setup

package_name = 'stepper_performan'

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
    maintainer='itmypi',
    maintainer_email='itmypi@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'plot_performan = stepper_performan.plot_performan:main',
            'stepper_test_cli = stepper_performan.stepper_test_cli:main',
        ],
    },
)
