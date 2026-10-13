from setuptools import find_packages, setup

package_name = 'apollo_agv_sim'

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
    description='2D simulator of the Apollo AGV.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'sim2d = apollo_agv_sim.node:main',
        'run2d = apollo_agv_sim.run2d:main',
    ]},
)
