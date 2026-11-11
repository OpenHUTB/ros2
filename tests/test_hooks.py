"""Tests for the MkDocs copyright hook."""

import importlib.util
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("ros2_docs_hooks", ROOT / "hooks.py")
HOOKS = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(HOOKS)


def test_config_hook_formats_year():
    config = SimpleNamespace(copyright="OpenHUTB 版权所有 &copy; {year}")

    result = HOOKS.on_config(config)

    assert result is config
    assert config.copyright == f"OpenHUTB 版权所有 &copy; {datetime.now().year}"


def test_config_hook_preserves_missing_copyright():
    config = SimpleNamespace(copyright=None)

    result = HOOKS.on_config(config)

    assert result is config
    assert config.copyright is None


def test_config_hook_does_not_break_unsupported_placeholders():
    config = SimpleNamespace(copyright="Copyright {owner}, {year}")

    result = HOOKS.on_config(config)

    assert result is config
    assert config.copyright == "Copyright {owner}, {year}"
