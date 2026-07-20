# SOURCE.md — 数字溯源

本报告站每个主数字均可追溯到下列材料（云环境未能直连 LFT 时，以会话已核对的 V5.1 / Expert 包数字为准）。

## 主文档

| 文件 | 用途 |
|------|------|
| `EgoVid_annotate/docs/egovid_pipeline_v5_1_report_zh.md` | V5.1 中文定稿：最佳 Seg 0.2031 / E2E 0.1517 |
| `EgoVid_annotate/docs/egovid_pipeline_v5_1_report.md` | 英文版 |
| `EgoVid_annotate/docs/wgo_bench_v5_1_results.md` | 结果表 |
| `EgoVid_annotate/docs/wgo_s2_fullcover_prompt.md` | S2 full-cover prompt |
| `wgo_reports/EXPERT_REVIEW_PACKET.md` | Expert 交接全表 |
| `wgo_reports/FULL_PLAIN_LANGUAGE_REPORT.md` | 通俗讲解 + S1/S2 full25 |
| `wgo_reports/SCORING_WALKTHROUGH.md` | 计分玩具例子 |
| Macrodata blog | 对照 0.306 / 61% / 0.168；HomER-only Gemini≈0.227 |

## 关键 score JSON（LFT）

根：`.../WGO_Bench_HomER/scores/`

| 实验 | 典型文件 |
|------|----------|
| aligned GEPA | `expB_recipe_aligned` 相关 segmentation json |
| S1 full25 | `expS1_anti_underseg_full25` |
| S2 early | `expS2_refine_full25` |
| **S2 fullcover 最强** | `expS2_refine_qwen36_27b_pad0_fullcover_prompt_segmentation.json` |
| **E2E 最强** | `expS2_fullcover_candidate_select397_labels_e2e_judge397.json` |
| 真 hand-crop 标注 | `expL2_true_hand_crop_full397_judge397.json` |
| Gemini flash | `exp_api_gemini25_flash` 系列 |

## 本机交付样例包

`wgo_reports/e2e_0.1517_homer25_labels/`（25 labels + videos）

## 对照说明（避免误读）

- 我们主表是 **HomER 25 / 470**，不是 blog 全量 100 / 743。
- blog headline Segment F1 **0.306** = Gemini + 全量 100；HomER-only Gemini ≈ **0.227** 才是更公平天花板。
- S1/S2 early full25 用 397B，aligned 用 27B，跨模型 Δ 含混淆；最终最强分段为 **同 recipe 下 Qwen3.6 S2 full-cover = 0.2031**。
- L4 掉分是 judge 变严，不是 caption 模型变差。
