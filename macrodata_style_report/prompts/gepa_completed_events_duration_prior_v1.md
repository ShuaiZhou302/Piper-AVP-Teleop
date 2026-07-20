# Prompt: completed_events_duration_prior_v1 (GEPA)

来源：Macrodata WGO-Bench blog（GEPA 搜索得到的最佳分段 prompt）。  
我们 `aligned` / 后续 S2 基座对齐此规则集。

```
Reconstruct the sequence of manipulation events in this robot video from the timestamped contact sheets.

Return only JSON with this shape:
{"segments":[{"start_sec":0.0,"end_sec":1.0,"subtask":"short action description"}]}

Rules:
- Segment only completed robot manipulation events, not every visible movement.
- Good boundaries happen when a held object changes, an object is placed or released, a tool starts/stops changing a surface, a container/door/lid opens or closes, or contents move between containers.
- Do not split approach, grasp adjustment, small repositioning, and retreat unless the world state changes.
- Do not merge separate pick/place/open/close/pour/wipe events when they complete different states.
- Most segments should be 2-10 seconds. Shorter segments are okay only for fast pick, place, open, close, or release events.
- Use the visible timestamps for start_sec and end_sec.
- Ignore label wording quality; prioritize temporally correct boundaries.
```

## 我们实现时的附加约定

- Contact sheet：约 **0.5s** 采样、**224px** tile、约 **20** 帧/sheet、黄字可视时间戳。
- **整集一次**：尽量一次送入该 episode 全部 sheets（`max_sheets_per_call=0`），避免分片接缝假切点。
- 输出解析为秒级区间；过滤空/FAILED 假段后再打 Segment F1@0.75 + outer snap。
