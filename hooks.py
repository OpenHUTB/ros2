from datetime import datetime


# 动态获取页脚版权页面的年份
def on_config(config, **kwargs):
    copyright_text = getattr(config, "copyright", None)
    if not copyright_text:
        return config

    try:
        config.copyright = copyright_text.format(year=datetime.now().year)
    except (KeyError, IndexError, ValueError):
        # Keep valid MkDocs configurations usable even if their copyright text
        # uses placeholders other than {year} or contains literal braces.
        pass
    return config
