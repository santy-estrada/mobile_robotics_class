from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'delta_ekf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'config'), glob('config/*.*')),
        (os.path.join('share', package_name,'launch'), glob('launch/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santy-estrada',
    maintainer_email='santiago.estrada6@eia.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekf_node = delta_ekf.ekf_node:main',
            # 'imu_stamper = delta_ekf.imu_stamper:main',
            'madgwick_filter_node = delta_ekf.madgwick_filter_node:main',
            'encoder_subs_node = delta_ekf.encoder_subs_node:main',
            'tf_ph_nostamp_node = delta_ekf.tf_ph_nostamp_node:main',
        ],
    },
)
