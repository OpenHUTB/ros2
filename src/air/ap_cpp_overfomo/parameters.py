"""OverFOMO 仿真流程的参数配置文件。

所有路径都由本文件所在目录（``PROJECT_ROOT``）拼接得出，不写死任何绝对路径，
因此把仓库克隆到任何位置、在任何一台机器上都能直接运行。

若确实要把数据放在仓库之外，请用环境变量覆盖，例如::

    set OVERFOMO_DATA_DIR=D:\\datasets\\overfomo
"""

import os

# -- Path parameters -- #
# 本文件所在目录就是工程根目录，其余路径一律相对它拼接。
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

# 允许用环境变量把大体积数据放到仓库外，默认仍为工程根目录。
DATA_ROOT = os.environ.get('OVERFOMO_DATA_DIR', PROJECT_ROOT)

# 输入 / 输出
json_path = os.path.join(PROJECT_ROOT, 'inputVariables.json')
save_path_wps = os.path.join(DATA_ROOT, 'results')
turnwps_path = os.path.join(PROJECT_ROOT, 'CPP', '002', 'TurnWPs.txt')

# -- Model parameters -- #
# 语义分割权重体积较大，未纳入版本库，见 README 的说明；
# 可用 OVERFOMO_WEIGHTS 环境变量指向实际位置。
weights_path = os.environ.get(
    'OVERFOMO_WEIGHTS', os.path.join(DATA_ROOT, 'weights0500.hdf5'))
backbone = 'efficientnetb1'

# -- Orthophoto parameters -- #
# 正射影像同样属于大体积数据，默认放在 DATA_ROOT 下；
# 没有这批影像时，纯规划流程（ap_cpp/、demos/、tests/）仍可完整运行。
ortho_georef_img = os.path.join(
    DATA_ROOT, 'ASLdataset', 'RedEdge', '002', 'reflectance-tif',
    'transparent_reflectance_red.tif')
ortho_rgb_img = os.path.join(
    DATA_ROOT, 'ASLdataset', 'RedEdge', '002', 'composite-png', 'RGB.png')

# Path planning parameters
QGIS = True  # read data from QGIS - GEOJSON file exported from QGIS
qgis_path = os.path.join(PROJECT_ROOT, 'CPP', '002', 'Polygon002.geojson')
mission_type = 'constant'  # 'constant'  # or 'variable'
initial_velocity = 4
time_interval = 0.5
distance_threshold = 0.3
if mission_type == 'constant':
    corner_radius = initial_velocity + 3
elif mission_type == 'variable':
    corner_radius = initial_velocity + 1 + 3  # if mission_type = 'variable': corner_radius = max_speed + 3

# How & Where to save your results
data_dir = save_path_wps  # save path
field = '002'  # name of the field
type_polygon = 'name_of_polygon'  # name of the polygon

save_path = os.path.join(data_dir, field, 'type_polygon_{}'.format(type_polygon), mission_type,
                         'time_interval_{}'.format(time_interval),
                         'velocity_{}'.format(initial_velocity))

# e.g. for the above parameters your results will be saved in a folder located in 'data_dir'/002/name_of_polygon/constant/time_interval_0.5/velocity_4
