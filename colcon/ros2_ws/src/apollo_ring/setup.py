from setuptools import find_packages, setup

package_name = 'apollo_ring'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pedro Sanchez',
    maintainer_email='pedro.sc.97@gmail.com',
    description='Ring stepper link to the Arduino: protocol, bridge node, bench tools.',
    license='Proprietary',
    tests_require=['pytest'],
    # ring_bridge and ring_cli entry points go here when those modules land.
    entry_points={'console_scripts': []},
)
