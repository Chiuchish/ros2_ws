import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'decentralized_formation'

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
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'visualizer_node=decentralized_formation.visualizer_color:main',
            'agent_node=decentralized_formation.decentralized:main',
            'control_node=decentralized_formation.new_motion_control:main',
            'central_node=decentralized_formation.central_node:main',
            'evaluator_node=decentralized_formation.data_analyst:main',
            'formation_manager_node=decentralized_formation.formation_manager:main',            
        ],
    },
)
