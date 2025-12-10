from setuptools import setup
pkg='rov_ctrl_core'
setup(name=pkg, version='0.1.0',
      packages=[pkg, f'{pkg}.nodes', f'{pkg}.utils'],
      data_files=[('share/ament_index/resource_index/packages',['resource/'+pkg]),('share/'+pkg,['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, entry_points={'console_scripts':[
        'mode_arming_node = rov_ctrl_core.nodes.mode_arming_node:main',
        'manual_control_node = rov_ctrl_core.nodes.manual_control_node:main',
        'safety_failsafe_node = rov_ctrl_core.nodes.safety_failsafe_node:main',
      ]})
