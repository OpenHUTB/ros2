# -*- coding: utf-8 -*-
"""诊断：行尾/连字符/ffmpeg/分支差距"""
import subprocess

mods = {
    'mujoco_go2': ['src/ground/go2_mujoco/main.py', 'src/ground/go2_mujoco/common.py',
                   'src/ground/go2_mujoco/README.md', 'src/ground/go2_mujoco/requirements.txt',
                   'docs/ground/go2_mujoco_simulation.md'],
    'mujoco_panda': ['src/manipulator/panda_mujoco/main.py', 'src/manipulator/panda_mujoco/common.py',
                     'src/manipulator/panda_mujoco/README.md', 'src/manipulator/panda_mujoco/requirements.txt',
                     'docs/manipulator/panda_mujoco_simulation.md'],
    'mujoco_shadow_hand': ['src/manipulator/shadow_hand_mujoco/main.py', 'src/manipulator/shadow_hand_mujoco/common.py',
                           'src/manipulator/shadow_hand_mujoco/README.md', 'src/manipulator/shadow_hand_mujoco/requirements.txt',
                           'docs/manipulator/shadow_hand_mujoco_simulation.md'],
}
out = []
for b, fs in mods.items():
    for f in fs:
        data = subprocess.run(['git', 'show', b + ':' + f], capture_output=True).stdout
        crlf = data.count(b'\r\n')
        lf = data.count(b'\n')
        bad = [c for c in data.decode('utf-8', 'ignore') if ord(c) == 0x2011]
        out.append('%s %s | CRLF %d / LF %d | bad-hyphen %d' % (b, f, crlf, lf, len(bad)))

# mkdocs.yml 与 index.md 是否被我改坏了行尾（对比 origin/master）
for f in ('mkdocs.yml', 'docs/index.md'):
    base = subprocess.run(['git', 'show', 'origin/master:' + f], capture_output=True).stdout
    cur = subprocess.run(['git', 'show', 'mujoco_go2:' + f], capture_output=True).stdout
    out.append('%s | base CRLF %d -> branch CRLF %d' % (f, base.count(b'\r\n'), cur.count(b'\r\n')))

# ffmpeg 是否存在
try:
    v = subprocess.run(['ffmpeg', '-version'], capture_output=True, text=True, timeout=30).stdout.splitlines()
    out.append('ffmpeg: ' + v[0])
except Exception as e:
    out.append('ffmpeg missing: ' + str(e))

# 上游 master 与各分支的差距
subprocess.run(['git', 'remote', 'add', 'upstream', 'https://github.com/OpenHUTB/ros2.git'], capture_output=True)
subprocess.run(['git', 'fetch', 'upstream', 'master'], capture_output=True, timeout=300)
for b in ('mujoco_go2', 'mujoco_panda', 'mujoco_shadow_hand'):
    behind = subprocess.run(['git', 'rev-list', '--count', b + '..upstream/master'], capture_output=True, text=True).stdout.strip()
    ahead = subprocess.run(['git', 'rev-list', '--count', 'upstream/master..' + b], capture_output=True, text=True).stdout.strip()
    out.append('%s | behind upstream/master: %s | ahead: %s' % (b, behind, ahead))

open('_diag.txt', 'w', encoding='utf-8').write('\n'.join(out))
print('\n'.join(out))
