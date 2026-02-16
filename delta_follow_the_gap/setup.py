from setuptools import find_packages, setup

package_name = 'delta_follow_the_gap'

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
            'control_gap_ttc = delta_follow_the_gap.control_gap_ttc:main',
            'ttc_gap_logger_node = delta_follow_the_gap.ttc_gap_logger_node:main',
        ],
    },
)
