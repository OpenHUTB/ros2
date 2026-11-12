from glob import glob
from setuptools import setup

setup(name='telemetry_imputation', version='0.1.0', packages=['telemetry_imputation'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/telemetry_imputation']),
                  ('share/telemetry_imputation', ['package.xml']),
                  ('share/telemetry_imputation/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='AipotuY', maintainer_email='283521505+AipotuY@users.noreply.github.com',
      description='Masked GRU reconstruction of dropped UAV telemetry samples', license='MIT',
      entry_points={'console_scripts': [
          'impute_replay=telemetry_imputation.ros_nodes:replay',
          'impute_estimator=telemetry_imputation.ros_nodes:imputer',
          'impute_dashboard=telemetry_imputation.dashboard:main']})
