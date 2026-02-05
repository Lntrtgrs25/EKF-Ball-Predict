from setuptools import find_packages, setup

package_name = 'EKF-Ball-Predict'

setup(
    name=package_name,
    version='0.0.0',
    packages=['EKF-Ball-Predict'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mbsaloka',
    maintainer_email='mbsaloka@gmail.com',
    description='Simulation of MCL for soccer using PyGame and ROS2',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = soccer_mcl_sim.sim:main',
            'config = soccer_mcl_sim.config_gui:main',
        ],
    },
)
