from setuptools import find_packages, setup
from glob import glob

package_name = 'ur_robotiq'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='penzottimattia@gmail.com',
    description='Bimanual UR3 + Robotiq 2F-85 integration package for ROS 2 Jazzy.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bimanual_setup_node = ur_robotiq.bimanual_setup_node:main',
            'export_unit_assets = ur_robotiq.export_unit_assets:main',
            'compute_camera_to_base = ur_robotiq.compute_camera_to_base:main',
            'hand_eye_calibration = ur_robotiq.hand_eye_calibration:main',
        ],
    },
)
