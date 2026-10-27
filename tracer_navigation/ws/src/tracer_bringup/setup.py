from setuptools import setup
setup(name='tracer_bringup',version='0.3.0',packages=['tracer_bringup'],
      data_files=[('share/ament_index/resource_index/packages',['resource/tracer_bringup']),
                  ('share/tracer_bringup',['package.xml']),('share/tracer_bringup/launch',['launch/system.launch.py','launch/visualization.launch.py'])],
      install_requires=['setuptools'],zip_safe=True,maintainer='TRACER team',maintainer_email='local@example.invalid',
      description='ROS2 unified mapping and navigation',license='Apache-2.0',
      entry_points={'console_scripts':[
          'mapping_backend = tracer_bringup.entry:mapping_backend',
          'lio_bridge = tracer_bringup.entry:lio_bridge',
          'model_adapter = tracer_bringup.entry:model_adapter',
          'model_recorder = tracer_bringup.entry:model_recorder',
          'visual_demo = tracer_bringup.entry:visual_demo']})
