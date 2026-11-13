from setuptools import setup

package_name = 'vda5050_door_adapter'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config.yaml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='Robot Control System',
    maintainer_email='dev@example.com',
    description='VDA 5050 door adapter for Open-RMF: publishes door_states and grants door_requests.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
