from setuptools import setup
import os, glob

pkg = 'rov_ctrl_alg_template'
setup(
    name=pkg,
    version='0.1.0',
    packages=[pkg, f'{pkg}.nodes', f'{pkg}.algorithms'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+pkg]),
        ('share/'+pkg, ['package.xml']),
        # ▼ install semua YAML ke share/<pkg>/config
        ('share/'+pkg+'/config', glob.glob('config/*.yaml')),
        # (opsional) install launch files
        ('share/'+pkg+'/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'alg_runner = rov_ctrl_alg_template.nodes.alg_runner:main',
        'alg_runner_gmc = rov_ctrl_alg_template.nodes.alg_runner_gmc:main',
    ]},
)
