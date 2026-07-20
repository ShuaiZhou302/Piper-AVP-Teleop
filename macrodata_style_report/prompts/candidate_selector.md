# Prompt: Candidate Selector（E2E 0.1517）

来源：V5.1 Relabeling & Candidate Selection。  
边界锁死为 S2 full-cover（Segment F1=0.2031）；对每个 pred 段收集多路 caption 候选，用 397B 选一个。

## 候选来源（消融里出现过）

- 分段模型 self label  
- Qwen3.5-397B raw frames  
- 397B ffmpeg 抽帧 raw  
- neighbor-context relabel（28B prior / raw397 prior）

## Selector 骨架

```
You are selecting the best subtask label for a fixed video segment.

Segment time: [{start_sec}, {end_sec}]
Episode instruction: {instruction}

Candidates:
{numbered_candidate_list}

Pick the single candidate that best names the completed manipulation event visible in the segment.
Prefer concrete object + action + destination/state. Prefer candidates that do not invent unseen objects.
If several are equivalent, prefer the more concise imperative phrasing (often the raw397 candidate).

Return JSON only:
{"selected_index": 0, "label": "the chosen label text", "reason": "one short sentence"}
```

## 结果（HomER）

| 变体 | E2E F1 |
|------|--------|
| self labels | 0.1054 |
| + raw397 | 0.1388 |
| + ffmpeg raw397 | 0.1414 |
| + neighbor | 0.1080–0.1440 |
| **+ candidate selector** | **0.1517** |
