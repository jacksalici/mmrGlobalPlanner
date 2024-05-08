from setuptools import setup

package_name = 'global_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name +'/config', ['config/params.yaml']),
        ('share/' + package_name +'/launch', ['launch/global_planner.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jacksalici',
    maintainer_email='',
    description='Global Planner Node for MMR Driverless',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner = global_planner.global_planner_node:main',
        ]

    },
)
