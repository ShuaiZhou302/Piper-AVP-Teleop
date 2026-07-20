# HomER Macrodata 风格长报告站

## 打开方式

```bash
cd /opt/cursor/artifacts/macrodata_style_report
python3 -m http.server 8765
# 浏览器打开 http://127.0.0.1:8765/
```

也可直接打开 `index.html`（已内嵌 `results.json` 与 prompts，file:// 可用）。

镜像副本：`/workspace/macrodata_style_report/`（实现后请再同步一次最新 artifacts）。

## 内容

- 全部分段 / 标注 / E2E 消融表与条形图
- 指标 walkthrough + 时间轴示意
- Contact sheet / GEPA / S1 / S2 / selector 通俗讲解
- Prompt 全文附录
- `SOURCE.md` 数字溯源

## 最佳数字（HomER 25/470）

| 指标 | 分数 | 配置 |
|------|------|------|
| Segment F1 | 0.2031 | S2 pad=0 full-cover · Qwen3.6 |
| Label Acc | 51.1%（raw 50.6%） | 真 HaWoR crop / raw · 397B |
| E2E F1 | 0.1517 | 上列边界 + 397B candidate selector |
