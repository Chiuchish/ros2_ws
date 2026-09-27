import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'centralized_op'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryan',
    maintainer_email='ryanchiu0601@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'visualizer_node=centralized_op.visualizer_color:main',
            'agent_node=centralized_op.decentralized:main',
            'control_node=centralized_op.dynamic_control:main',
            'central_node=centralized_op.central_node:main',
            'evaluator_node=centralized_op.data_analyst:main'
        ],
    },
)
