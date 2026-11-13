from setuptools import setup

package_name = 'vda5050_fleet_adapter'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config.yaml']),
        ('share/' + package_name, ['nav_graph.yaml']),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='Robot Control System',
    maintainer_email='dev@example.com',
    description='VDA 5050 full_control fleet adapter for Open-RMF (ADR-004).',
    license='Apache-2.0',
    tests_require=['pytest'],
)
