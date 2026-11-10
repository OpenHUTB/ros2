from setuptools import setup

setup(name='turtlebot3_relocalization', version='0.1.0',
      py_modules=['reloc_core', 'turtlebot3_amcl'],
      data_files=[('share/ament_index/resource_index/packages',
                   ['resource/turtlebot3_relocalization']),
                  ('share/turtlebot3_relocalization', ['package.xml'])],
      install_requires=['setuptools'],
      maintainer='zqhuang76', maintainer_email='zqhuang76@users.noreply.github.com',
      description='TurtleBot3 AMCL-style particle filter localization and relocalization',
      license='Apache-2.0',
      entry_points={'console_scripts': [
          'turtlebot3_amcl = turtlebot3_amcl:main',
          'relocalization_2d = main:main']})
