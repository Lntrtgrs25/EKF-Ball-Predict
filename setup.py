from setuptools import find_packages, setup

package_name = 'soccer_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=['soccer_sim'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mbsaloka',
    maintainer_email='mbsaloka@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'soccer_sim = soccer_sim.sim:main',
        ],
    },
)
