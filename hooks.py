from datetime import datetime


# 动态获取页脚版权页面的年份
def on_config(config, **kwargs):
    copyright_text = config.copyright
    if isinstance(copyright_text, str):
        config.copyright = copyright_text.replace("{year}", str(datetime.now().year))
