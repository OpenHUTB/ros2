from glob import glob

from setuptools import find_packages, setup

package_name = 'humanoid_voice_command'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'scripts']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'main.py', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # 意图网络（304 KB，TorchScript）
        ('share/' + package_name + '/weights', glob('weights/*.pt')),
        # Whisper 权重不在仓库里（72 MB），默认运行时下载到 ~/.cache/whisper；
        # 若本地放了一份（scripts/download_weights.py --copy-to）则一并安装，断网可用
        ('share/' + package_name + '/weights/whisper', glob('weights/whisper/*.pt')),
        # 预录口令音频（18 条，约 1 MB，避免评测机还要装 TTS）
        ('share/' + package_name + '/audio', glob('audio/*.wav')),
    ],
    install_requires=[
        'setuptools>=30.3.0,<80',
        'numpy<3',
        'pyyaml',
        # 语音识别依赖，故意不写死版本以免 pip 拉取 CUDA 版 torch：
        #   pip install torch --index-url https://download.pytorch.org/whl/cpu
        #   pip install openai-whisper soundfile
        # 'torch>=2.0',
        # 'openai-whisper>=20231117',
        # 'soundfile>=0.12',
    ],
    zip_safe=True,
    maintainer='OpenHUTB contributor',
    maintainer_email='maintainer@example.com',
    description=(
        'Offline voice command control for humanoid robots: Whisper tiny speech '
        'recognition plus a rule/MLP intent classifier driving /cmd_vel.'
    ),
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'voice_node = humanoid_voice_command.voice_node:main',
            'command_node = humanoid_voice_command.command_node:main',
        ],
    },
)
