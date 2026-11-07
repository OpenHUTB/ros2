import os

import yaml


def _root():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_params_file_exists():
    assert os.path.isfile(os.path.join(_root(), 'config', 'params.yaml'))


def test_params_loads_as_dict():
    with open(os.path.join(_root(), 'config', 'params.yaml'), 'r', encoding='utf-8') as fh:
        cfg = yaml.safe_load(fh)
    assert isinstance(cfg, dict) and cfg