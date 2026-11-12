from setuptools import setup

setup(name='jetbot_semantic_nav',version='0.1.0',packages=['jetbot_semantic_nav'],
      data_files=[('share/ament_index/resource_index/packages',['resource/jetbot_semantic_nav']),
                  ('share/jetbot_semantic_nav',['package.xml']),
                  ('share/jetbot_semantic_nav/config',['config/object_observation.schema.json']),
                  ('share/jetbot_semantic_nav/launch',['launch/semantic_nav.launch.py'])],
      install_requires=['setuptools'],zip_safe=True,
      maintainer='JetBot navigation team',maintainer_email='nav@example.invalid',
      description='Simulation oracle semantic object navigation',license='Apache-2.0',
      entry_points={'console_scripts':['semantic_mapper=jetbot_semantic_nav.mapper:main',
          'semantic_goal_manager=jetbot_semantic_nav.manager:main',
          'semantic_command_cli=jetbot_semantic_nav.cli:main']})
