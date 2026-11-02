# 文档工具与导出验收 / Documentation tool acceptance

## hutb-doc

本项目使用 `hutb-doc 1.5.9`（MkDocs 兼容发行版）构建站点，输出目录为 `site-hutb-doc-check`。项目页面已生成，GitHub Pages 在线地址为：
`https://cheng859348.github.io/ros2/`

## Markdown 转换与 Word

Markdown 内容已通过 Markdown 转换器导出为 Word，并完成 Word 打开、保存、关闭、重新打开验收。中英文标题、公式说明、运行步骤和测试结果均可回读。

## Training and neural planning

CNN training is provided by `train.py`; synchronized navigation uses the MLP policy in `neural_navigation.py`. The runtime loop updates SLAM and emits the neural control action in the same simulation step.
