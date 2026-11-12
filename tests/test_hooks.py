"""hooks.on_config：copyright 年份替换的健壮性测试。

覆盖 mkdocs 中 copyright 可能出现的几种取值：
  - 正常 {year} 占位符
  - 含 {year} 以外的占位符（旧实现会抛 KeyError）
  - 未配置（None，旧实现会抛 AttributeError）
  - 非字符串
  - 不含占位符
"""
import types
from datetime import datetime

import hooks

_YEAR = str(datetime.now().year)


def _config(copyright):
    """构造一个最小的 mkdocs config 替身，只提供 copyright 属性。"""
    return types.SimpleNamespace(copyright=copyright)


def test_replaces_year_placeholder():
    cfg = _config("OpenHUTB 版权所有 &copy; {year}")
    hooks.on_config(cfg)
    assert cfg.copyright == "OpenHUTB 版权所有 &copy; %s" % _YEAR


def test_keeps_other_placeholders():
    """{org} 这类非年份占位符应原样保留，不应抛 KeyError。"""
    cfg = _config("Copyright {org} {year}")
    hooks.on_config(cfg)
    assert cfg.copyright == "Copyright {org} %s" % _YEAR


def test_none_is_ignored():
    """copyright 未配置（None）时不应抛异常。"""
    cfg = _config(None)
    hooks.on_config(cfg)
    assert cfg.copyright is None


def test_non_str_is_ignored():
    """copyright 非字符串时原样返回，不应抛 AttributeError。"""
    cfg = _config(2026)
    hooks.on_config(cfg)
    assert cfg.copyright == 2026


def test_without_placeholder_unchanged():
    cfg = _config("OpenHUTB 版权所有")
    hooks.on_config(cfg)
    assert cfg.copyright == "OpenHUTB 版权所有"
