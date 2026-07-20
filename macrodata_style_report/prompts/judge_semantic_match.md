# Prompt: LLM Judge（语义是否同一完成事件）

来源：`judge_labels.py` 默认 JUDGE_PROMPT（文本-only）。  
**Match 不是规则 parse**，而是再调一次 LLM。

## 默认（相对宽松）

```
You are judging whether a predicted subtask label describes the SAME completed manipulation event as the gold label.

Gold: {gold}
Pred: {pred}

A match is true if they share the same verb family AND the same primary object/target (destination/state), even if wording differs.

Reply JSON only:
{"match": true|false, "reason": "one short sentence"}
```

## L4 更严变体（重判同一批 raw → Acc 50.6%→43.0%）

更强调：物体身份、左右侧、精确目标位置、不得把不同 pick/place 合成一次。用于评测口径消融，**不当成模型涨分**。

## E2E 用法

1. 先按 IoU@0.75 + outer snap + greedy 1-1 做时间配对。  
2. 仅对配对成功的 (pred, gold) 送 judge。  
3. semantic match 数进入 E2E P/R/F1。
