from setuptools import find_packages, setup

package_name = 'turtlebot3_navcontrol'

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
    maintainer='apollo',
    maintainer_email='stevbach@udel.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_navcontrol = turtlebot3_navcontrol.turtlebot3_navcontrol:main',
        ],
    },
)
