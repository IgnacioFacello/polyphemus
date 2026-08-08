from setuptools import find_packages, setup

package_name = 'control_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pygame'],
    zip_safe=True,
    maintainer='ifacello',
    maintainer_email='ignacio.facello@mi.unc.edu.ar',
    description='Python node that controls the robot´s motors',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'display = control_service.display_node:main',
        ],
    },
)
