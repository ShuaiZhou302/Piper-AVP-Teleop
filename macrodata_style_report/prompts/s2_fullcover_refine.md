# Prompt: S2 full-cover 局部精修（当前最强分段）

来源：`EgoVid_annotate/docs/wgo_s2_fullcover_prompt.md` + V5.1 定稿。  
配置：**pad_sec=0**，短窗 contact sheet 覆盖粗段，要求模型**只重切该窗**。

## 人话

1. 先用整集 + GEPA 得到粗边界。  
2. 对每个粗段（或滑动覆盖窗）生成局部 contact sheet（约 0.5s×20 帧 ≈ 10s 视野）。  
3. 把粗段起止作为 hint 告诉模型：在这个时间窗内重新切出更细的 completed events。  
4. `pad=0`：不加额外前后填充（消融里 pad=0.5/1/2 未超过 pad=0）。

## Prompt 骨架（复现用）

```
You are refining subtask boundaries inside ONE local time window of an egocentric video.

You are given:
1) Timestamped contact-sheet tiles for this window only (read the yellow timestamps).
2) A coarse segment hint: [{coarse_start_sec}, {coarse_end_sec}] with optional coarse label.

Task:
- Re-segment ONLY events that fall inside this window.
- Follow completed-events rules (GEPA): completed manipulations only; do not split approach/adjust/retreat without world-state change; do not merge distinct pick/place/open/close/...
- Prefer segments roughly 2-10s unless a fast atomic event is clearly shorter.
- Output JSON only:
{"segments":[{"start_sec":0.0,"end_sec":1.0,"subtask":"..."}]}

Rules:
- start_sec/end_sec must use visible tile timestamps and stay within the window.
- Do not invent boundaries only because the contact sheet starts or ends.
- If the coarse hint already matches one completed event, you may keep a single segment.
- If the coarse hint merges multiple completed events, split them.
- Ignore wording quality; prioritize correct temporal boundaries.
```

## 关键参数

| 项 | 值 |
|----|-----|
| 模型 | Qwen3.6-27B/28B |
| pad_sec | 0 |
| sheet | 0.5s, 224px, ~20 tiles |
| HomER Segment F1 | **0.2031**（79/308/470） |
