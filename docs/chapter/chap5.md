# 5. 机器人感知

本章示例代码位于 `src/chap5/`。

## 5.1 行人隐私脱敏视觉检测节点

- [行人隐私脱敏视觉检测节点](privacy_mask/README.md)：基于 YOLOv8n 检测静态图片中的行人，
  对行人包围盒做高斯模糊，并发布 `/image_raw/compressed` 与 `/image_masked/compressed` 两个压缩图像话题。
