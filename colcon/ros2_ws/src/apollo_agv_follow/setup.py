from setuptools import find_packages, setup

package_name = 'apollo_agv_follow'

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
    description='Follow guidance for the Apollo AGV.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['follower = apollo_agv_follow.node:main']},
)
