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


def _current_year():
    return str(datetime.now().year)


def test_config_hook_formats_year():
    config = SimpleNamespace(copyright="OpenHUTB 版权所有 &copy; {year}")

    result = HOOKS.on_config(config)

    assert result is config
    assert config.copyright == f"OpenHUTB 版权所有 &copy; {_current_year()}"


def test_config_hook_preserves_missing_or_non_string_copyright():
    for value in (None, 2026):
        config = SimpleNamespace(copyright=value)

        result = HOOKS.on_config(config)

        assert result is config
        assert config.copyright == value


def test_config_hook_updates_year_beside_other_placeholders():
    config = SimpleNamespace(copyright="Copyright {owner}, {year}")

    result = HOOKS.on_config(config)

    assert result is config
    assert config.copyright == f"Copyright {{owner}}, {_current_year()}"


def test_config_hook_preserves_escaped_year_placeholder():
    config = SimpleNamespace(copyright="Literal {{year}}")

    result = HOOKS.on_config(config)

    assert result is config
    assert config.copyright == "Literal {{year}}"


def test_config_hook_preserves_attribute_access():
    config = SimpleNamespace(copyright="Copyright {year.upper}")

    result = HOOKS.on_config(config)

    assert result is config
    assert config.copyright == "Copyright {year.upper}"


def test_config_hook_preserves_item_access():
    config = SimpleNamespace(copyright="Copyright {year[0]}")

    result = HOOKS.on_config(config)

    assert result is config
    assert config.copyright == "Copyright {year[0]}"


def test_config_hook_preserves_unclosed_braces():
    config = SimpleNamespace(copyright="Copyright {year")

    result = HOOKS.on_config(config)

    assert result is config
    assert config.copyright == "Copyright {year"
