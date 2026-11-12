"""打包一致性检查：不装 ROS 也能发现 colcon build 前后最常见的几类错误.

检查对象是"人写的元数据"而不是算法：
- package.xml 能不能解析、名字/许可/维护者/构建类型对不对；
- setup.py 的 entry point 与 data_files 与仓库里的文件是否对得上；
- launch 里调用的可执行名与 CLI 参数，是否都在 setup.py / cli.py 里有定义；
- 文档与 CI 引用的路径是否存在。

这些错误在 `colcon build` 时才会暴露，而本地又没有 ROS，所以先在这里拦住。
"""
import ast
import json
import re
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
NAME = 'rov_ballast_trim'


def read(path):
    # utf-8-sig：Windows 编辑器/PowerShell 生成的 UTF-8 BOM 不该让自检挂掉
    return Path(path).read_text(encoding='utf-8-sig')


def test_package_xml_is_well_formed():
    root = ET.parse(PACKAGE / 'package.xml').getroot()
    assert root.tag == 'package'
    assert root.get('format') == '3'
    assert root.findtext('name') == NAME
    assert root.findtext('version')
    assert root.findtext('license') == 'MIT'
    maintainer = root.find('maintainer')
    assert maintainer is not None and maintainer.text
    assert '@' in (maintainer.get('email') or '')
    assert root.findtext('description').strip()
    build_type = root.find('export/build_type')
    assert build_type is not None and build_type.text == 'ament_python'
    assert root.find('buildtool_depend') is not None


def test_setup_py_entry_point_matches_cli():
    source = read(PACKAGE / 'setup.py')
    ast.parse(source)  # 语法正确
    assert 'name=package_name' in source
    assert "'%s'" % NAME in source
    match = re.search(r"'(ballast_trim)\s*=\s*([\w.]+):(\w+)'", source)
    assert match, 'setup.py 里找不到 console_scripts 定义'
    executable, module, function = match.groups()
    assert executable == 'ballast_trim'
    assert module == '%s.cli' % NAME
    # 入口函数真实存在
    cli_source = read(PACKAGE / NAME / 'cli.py')
    assert ('def %s(' % function) in cli_source


def test_data_files_exist():
    for name in ('package.xml', 'README.md', 'main.py'):
        assert (PACKAGE / name).is_file(), 'data_files 缺文件: %s' % name
    launch_files = list((PACKAGE / 'launch').glob('*.launch.py'))
    assert launch_files, 'launch/ 下没有 *.launch.py'
    assert (PACKAGE / 'resource' / NAME).is_file()
    assert (PACKAGE / 'setup.cfg').is_file()
    # launch 在找不到 console script 时会退回 `python -m <pkg>`，必须有 __main__.py
    assert (PACKAGE / NAME / '__main__.py').is_file()
    cfg = read(PACKAGE / 'setup.cfg')
    assert 'lib/%s' % NAME in cfg


def test_launch_arguments_exist_in_cli():
    launch = read(PACKAGE / 'launch' / 'main.launch.py')
    cli_source = read(PACKAGE / NAME / 'cli.py')
    cli_flags = set(re.findall(r"add_argument\('(--[a-z-]+)'", cli_source))
    launch_flags = set(re.findall(r"'(--[a-z-]+)'", launch))
    assert 'ballast_trim' in launch, 'launch 没有调用 console script'
    unknown = launch_flags - cli_flags
    assert not unknown, 'launch 传了 CLI 不认识的参数: %s' % sorted(unknown)
    # launch 声明的每个 launch 参数都应该真的被用到
    declared = set(re.findall(r"\('([a-z_]+)',", launch))
    used = set(re.findall(r"LaunchConfiguration\('([a-z_]+)'\)", launch))
    # 可选参数走的是循环：LaunchConfiguration(name)，把 (name, '--flag') 表也计入
    used |= {name for name, _flag in re.findall(r"\('([a-z_]+)', '(--[a-z-]+)'\)", launch)}
    assert declared == used, 'launch 参数声明与使用不一致: %s' % sorted(declared ^ used)


def test_validation_json_matches_code():
    data = json.loads(read(PACKAGE / 'validation' / 'offline_validation.json'))
    assert data['result']['tests_passed'] >= 18
    assert data['result']['pass'] is True
    assert data['result']['required_ballast_kg'] == 38.505
    assert data['result']['gm_after_m'] == 0.0421
    # 没有 ROS 环境的机器上这两项不能凭空写成已验证；本包的 ROS 验证结果由 CI 回填
    assert isinstance(data['ros_validation']['checked'], bool)
    assert data['independent_machine_review'] is False


def test_docs_and_ci_reference_existing_paths():
    doc = REPO / 'docs' / 'water' / ('%s.md' % NAME)
    assert doc.is_file(), '缺少教程页'
    text = read(doc)
    for image in re.findall(r'!\[[^\]]*\]\(([^)]+)\)', text):
        assert (doc.parent / image).resolve().is_file(), '图片不存在: %s' % image
    nav = read(REPO / 'mkdocs.yml')
    assert './water/%s.md' % NAME in nav
    index = read(REPO / 'docs' / 'index.md')
    assert './water/%s.md' % NAME in index
    workflow = REPO / '.github' / 'workflows' / 'rov_ballast_trim_tests.yml'
    assert workflow.is_file()
    workflow_text = read(workflow)
    assert 'src/water/%s/**' % NAME in workflow_text
    # 只要求出现 colcon 构建该包，不绑定参数顺序（--base-paths 可能插在中间）
    assert 'colcon build' in workflow_text
    assert '--packages-select %s' % NAME in workflow_text


def main():
    failures = []
    tests = [(name, obj) for name, obj in sorted(globals().items())
             if name.startswith('test_') and callable(obj)]
    for name, func in tests:
        try:
            func()
        except AssertionError as error:
            failures.append('%s: %s' % (name, error))
            print('FAIL %s' % name)
        else:
            print('ok   %s' % name)
    print()
    print('%d passed, %d failed' % (len(tests) - len(failures), len(failures)))
    for item in failures:
        print('  - ' + item)
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
