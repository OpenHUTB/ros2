from datetime import datetime


# 动态获取页脚版权页面的年份
def on_config(config, **kwargs):
    year = str(datetime.now().year)
    config.copyright = config.copyright.format(year=year)
