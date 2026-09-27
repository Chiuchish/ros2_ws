import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'consensus_test'

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
        	'control_node=consensus_test.control_node:main',
        	'bayesian_auction_node=consensus_test.bayesian_auction_node:main',
        	'visualizer_node=consensus_test.visualizer_node:main',
            'bayesian_node=consensus_test.bayesian_node:main',
            'control_limit_node=consensus_test.control_limit:main',
            'Parameterized_bayesian_node=consensus_test.bayesian_para:main'
        ],
    },
)
