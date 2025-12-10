from setuptools import setup
pkg='rov_ctrl_demos'
setup(name=pkg, version='0.1.0', packages=[pkg,f'{pkg}.nodes'],
      data_files=[('share/ament_index/resource_index/packages',['resource/'+pkg]),('share/'+pkg,['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts':['simple_forward_demo = rov_ctrl_demos.nodes.simple_forward_demo:main']})
