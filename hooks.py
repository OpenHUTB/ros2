import re
from datetime import datetime


# 动态获取页脚版权页面的年份
def on_config(config, **kwargs):
    copyright_text = getattr(config, "copyright", None)
    if not isinstance(copyright_text, str):
        return config

    # Replace only an explicit {year}; escaped braces and unsupported
    # placeholders remain unchanged so malformed templates cannot break builds.
    year = str(datetime.now().year)
    config.copyright = re.sub(r"(?<!\{)\{year\}(?!\})", year, copyright_text)
    return config
