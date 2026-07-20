# Prompt: 固定边界 Labeling（Track A）

来源：Macrodata labeling prompt + 我们 `label_gold_segments.py` 实践。  
边界固定为 gold；只生成 label。

```
Annotate the fixed robot video segment shown in the contact sheet / frames.

Return only JSON:
{"label":"short descriptive subtask label"}

Focus on the state change caused by the segment.

Rules:
- The frames are chronological and timestamped (when timestamps are drawn).
- The segment boundaries are fixed; do not create, split, merge, or move segments.
- Compare the beginning and end of the segment, then describe the completed visible change.
- Use one concise imperative phrase.
- Name the manipulated object and the action/state change.
- Include source, destination, side, direction, final placement, opened/closed state, filled/cleaned/cut/drawn/folded part when visible.
- If the segment is a continuous process, describe the process and its target.
- Do not mention timestamps, frame numbers, uncertainty, or invisible intent.

Episode instruction: {instruction}
```

## 我们测过的视觉输入

| 输入 | HomER Acc | 备注 |
|------|-----------|------|
| raw 多帧 | 50.6% | 默认最强基线 |
| 光流 overlay proxy | 48.3% | 非真 hand overlay |
| 整帧 temporal collage | 42.1% | 非 Scale hand-collage |
| L1 邻段 sheet | 35.5–36.8% | 伤分 |
| L2 YOLO hand-collage | 40.6% | 伤分 |
| 真 HaWoR hand-crop | **51.1%** | 微涨 |
