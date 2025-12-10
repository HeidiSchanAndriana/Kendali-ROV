from setuptools import setup
package_name = 'rov_ctrl_if'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.nodes', f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/control_stack.launch.py']),
        ('share/' + package_name + '/config', ['config/control_mux.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Interface/mux for ROV control',
    license='MIT',
    entry_points={'console_scripts': [
        'control_mux = rov_ctrl_if.nodes.control_mux:main',
    ]},
)
