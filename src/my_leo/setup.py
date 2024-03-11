import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_leo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include URDF (.urdf) files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        # Include bt files
        (os.path.join('share', package_name, 'behavior_tree_xml'), glob(os.path.join('behavior_tree_xml', '*.xml'))),
        # Include rviz (.rviz) files
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        # Include world (.sdf or dae) files
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '**/**.[sd][da][fe]'), recursive=True)),
        # Include model (.sdf or dae) files
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.[sd][ta][le]'))),
        # Include config (.yaml) files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
	    # Include map (.yaml and .pgm) files
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.[yp][ag][m]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sfr2023',
    maintainer_email='sfr2023@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
