window.__RESULTS__ = {
  "meta": {
    "title": "EgoVid × WGO-Bench HomER 标注消融长报告",
    "eval_subset": "HomER 25 episodes / 470 gold segments",
    "metric_seg": "Segment F1@0.75 micro + outer snap",
    "metric_label": "LLM-judge accuracy on gold boundaries",
    "metric_e2e": "Semantic E2E F1 (IoU match then judge)",
    "blog_refs": {
      "full100_seg_f1": 0.306,
      "full100_label_acc": 0.61,
      "full100_e2e_f1": 0.168,
      "homer_only_gemini_seg_f1": 0.227,
      "opensource_same_recipe_seg": "0.11-0.14"
    },
    "best": {
      "seg_f1": 0.2031,
      "seg_config": "Qwen3.6 S2 pad=0 full-cover contact-sheet",
      "label_acc": 0.511,
      "label_config": "Qwen3.5-397B true HaWoR hand-crop (raw≈50.6%)",
      "e2e_f1": 0.1517,
      "e2e_config": "S2 full-cover boundaries + 397B candidate selector"
    },
    "generated_note": "Scores aggregated from V5.1 reports, EXPERT_REVIEW_PACKET, FULL_PLAIN_LANGUAGE_REPORT, and HomER score JSONs. HomER-only unless noted."
  },
  "segmentation": [
    {
      "id": "egovid_baseline",
      "name": "EgoVid 原管线（腕速切 + merge）",
      "f1": 0.0953,
      "p": null,
      "r": null,
      "match": 61,
      "pred": 810,
      "gold": 470,
      "model": "pipeline",
      "full25": true,
      "note": "过分割严重；启发式切段基线"
    },
    {
      "id": "cs_max3_397b",
      "name": "Contact sheet 旧B max_sheets=3",
      "f1": 0.0952,
      "p": 0.132,
      "r": 0.075,
      "match": 35,
      "pred": 265,
      "gold": 470,
      "model": "Qwen3.5-397B",
      "full25": true,
      "note": "分片调用在 sheet 接缝造假切点"
    },
    {
      "id": "cs_max3_27b",
      "name": "Contact sheet 旧B max_sheets=3",
      "f1": 0.1278,
      "p": 0.171,
      "r": 0.102,
      "match": 48,
      "pred": 281,
      "gold": 470,
      "model": "Qwen3.6-27B",
      "full25": true,
      "note": "同 recipe 下 27B 优于 397B 分片版"
    },
    {
      "id": "whole_legacy_27b",
      "name": "整集一次 + legacy prompt",
      "f1": 0.123,
      "p": 0.257,
      "r": 0.081,
      "match": 38,
      "pred": 148,
      "gold": 470,
      "model": "Qwen3.6-27B",
      "full25": true,
      "note": "欠分割：pred 过少"
    },
    {
      "id": "aligned_gepa_27b",
      "name": "整集一次 + GEPA（aligned）",
      "f1": 0.1369,
      "p": 0.228,
      "r": 0.098,
      "match": 46,
      "pred": 202,
      "gold": 470,
      "model": "Qwen3.6-27B",
      "full25": true,
      "note": "外形 recipe 对齐 blog；仍欠分割"
    },
    {
      "id": "s1_full25_397b",
      "name": "S1 反欠分割 full25",
      "f1": 0.1556,
      "p": 0.143,
      "r": 0.17,
      "match": 80,
      "pred": 558,
      "gold": 470,
      "model": "Qwen3.5-397B",
      "full25": true,
      "note": "召回升但过分割；含模型混淆 vs 27B aligned"
    },
    {
      "id": "s2_full25_397b",
      "name": "S2 二次精修 full25（早期）",
      "f1": 0.1674,
      "p": 0.163,
      "r": 0.172,
      "match": 81,
      "pred": 498,
      "gold": 470,
      "model": "Qwen3.5-397B",
      "full25": true,
      "note": "粗分后再局部细切"
    },
    {
      "id": "s2_fullcover_qwen36",
      "name": "S2 pad=0 + full-cover 局部 prompt",
      "f1": 0.2031,
      "p": 0.256,
      "r": 0.168,
      "match": 79,
      "pred": 308,
      "gold": 470,
      "model": "Qwen3.6-27B/28B",
      "full25": true,
      "note": "当前最强分段；contact sheet 0.5s×20≈10s 窗"
    },
    {
      "id": "gemini25_flash_api",
      "name": "AIHubMix gemini-2.5-flash API",
      "f1": 0.0721,
      "p": null,
      "r": null,
      "match": null,
      "pred": null,
      "gold": 470,
      "model": "gemini-2.5-flash",
      "full25": true,
      "note": "全量 seg 后低于开源；空 pred 曾需重试"
    }
  ],
  "labeling": [
    {
      "id": "raw_397b",
      "name": "raw 原帧（默认最强基线）",
      "acc": 0.506,
      "n_match": 238,
      "n": 470,
      "model": "Qwen3.5-397B",
      "full25": true,
      "note": "gold 边界固定；Track A"
    },
    {
      "id": "overlay_proxy",
      "name": "光流 overlay（optical_flow_proxy）",
      "acc": 0.483,
      "n_match": 227,
      "n": 470,
      "model": "Qwen3.5-397B",
      "full25": true,
      "note": "无 hand_recon；非真 hand overlay"
    },
    {
      "id": "temporal_collage",
      "name": "时序 collage（整帧 P/C/F）",
      "acc": 0.421,
      "n_match": 198,
      "n": 470,
      "model": "Qwen3.5-397B",
      "full25": true,
      "note": "≠ blog 邻段 sheet / Scale hand-crop"
    },
    {
      "id": "raw_27b",
      "name": "raw + 27B",
      "acc": 0.46,
      "n_match": 216,
      "n": 470,
      "model": "Qwen3.6-27B",
      "full25": true,
      "note": "小于 397B raw"
    },
    {
      "id": "l1_neighbor",
      "name": "L1 邻段 contact sheet",
      "acc": 0.368,
      "n_match": 173,
      "n": 470,
      "model": "Qwen3.5-397B",
      "full25": true,
      "note": "WGO 标注配方；本地 Qwen 上伤分"
    },
    {
      "id": "l1_ts_rerun",
      "name": "L1 + 秒级时间戳重跑",
      "acc": 0.355,
      "n_match": 167,
      "n": 470,
      "model": "Qwen3.5-397B",
      "full25": true,
      "note": "审计修缺时间戳后仍跌"
    },
    {
      "id": "l2_yolo_hand",
      "name": "L2 YOLO 腕点 hand-collage",
      "acc": 0.406,
      "n_match": 191,
      "n": 470,
      "model": "Qwen3.5-397B",
      "full25": true,
      "note": "禁 centerproxy；仍低于 raw"
    },
    {
      "id": "l2_hawor",
      "name": "真 HaWoR hand-recon crop",
      "acc": 0.511,
      "n_match": 240,
      "n": 470,
      "model": "Qwen3.5-397B",
      "full25": true,
      "note": "固定边界标注微涨 +0.4pp"
    },
    {
      "id": "l4_strict_judge",
      "name": "L4 严 judge 重判 raw",
      "acc": 0.43,
      "n_match": 202,
      "n": 470,
      "model": "397B judge",
      "full25": true,
      "note": "尺子变严≠模型变差"
    }
  ],
  "e2e": [
    {
      "id": "egovid_e2e",
      "name": "EgoVid one-pass baseline",
      "seg_f1": 0.0953,
      "e2e_f1": 0.0656,
      "pred_gold": "810/470",
      "note": "切段与标注一体弱"
    },
    {
      "id": "s2_self",
      "name": "28B S2 full-cover, self labels",
      "seg_f1": 0.2031,
      "e2e_f1": 0.1054,
      "pred_gold": "308/470",
      "note": "边界已锁；标注用分段模型自标"
    },
    {
      "id": "raw397",
      "name": "+ Qwen3.5-397B raw relabel",
      "seg_f1": 0.2031,
      "e2e_f1": 0.1388,
      "pred_gold": "308/470",
      "note": "边界固定，换大模型标"
    },
    {
      "id": "ffmpeg397",
      "name": "+ Qwen3.5-397B ffmpeg raw relabel",
      "seg_f1": 0.2031,
      "e2e_f1": 0.1414,
      "pred_gold": "308/470",
      "note": "抽帧路径变体"
    },
    {
      "id": "nb28",
      "name": "+ 28B-prior neighbor relabel",
      "seg_f1": 0.2031,
      "e2e_f1": 0.108,
      "pred_gold": "308/470",
      "note": "邻段上下文伤分"
    },
    {
      "id": "nb397",
      "name": "+ raw397-prior neighbor relabel",
      "seg_f1": 0.2031,
      "e2e_f1": 0.144,
      "pred_gold": "308/470",
      "note": "邻段略好于纯 self"
    },
    {
      "id": "selector397",
      "name": "+ Qwen3.5-397B candidate selector",
      "seg_f1": 0.2031,
      "e2e_f1": 0.1517,
      "pred_gold": "308/470",
      "note": "当前最强 E2E；多候选选优偏 raw"
    }
  ],
  "walkthrough_toy": {
    "gold": [
      {"id": "G0", "start": 0.0, "end": 3.0},
      {"id": "G1", "start": 3.0, "end": 6.0},
      {"id": "G2", "start": 6.0, "end": 10.0}
    ],
    "pred": [
      {"id": "P0", "start": 0.5, "end": 2.8},
      {"id": "P1", "start": 2.8, "end": 5.5},
      {"id": "P2", "start": 5.5, "end": 8.0},
      {"id": "P3", "start": 8.0, "end": 9.5}
    ],
    "after_snap": [
      {"id": "P0", "start": 0.0, "end": 2.8},
      {"id": "P1", "start": 2.8, "end": 5.5},
      {"id": "P2", "start": 5.5, "end": 8.0},
      {"id": "P3", "start": 8.0, "end": 10.0}
    ],
    "ious": {
      "P0-G0": 0.933,
      "P1-G1": 0.781
    },
    "n_match": 2,
    "P": 0.5,
    "R": 0.667,
    "F1": 0.571
  }
}
;


async function loadResults() {
  if (window.__RESULTS__) return window.__RESULTS__;
  const res = await fetch("data/results.json");
  return res.json();
}

function barChart(el, rows, key, maxVal, alt = false) {
  const max = maxVal || Math.max(...rows.map((r) => r[key] || 0), 0.01);
  el.innerHTML = rows
    .map((r) => {
      const v = r[key] ?? 0;
      const pct = Math.max(2, (v / max) * 100);
      const label = r.name.length > 42 ? r.name.slice(0, 40) + "…" : r.name;
      return `<div class="bar-row">
        <div class="bar-label" title="${r.name}">${label}</div>
        <div class="bar-track"><div class="bar-fill ${alt ? "alt" : ""}" style="width:${pct}%"></div></div>
        <div class="bar-val">${typeof v === "number" ? v.toFixed(3) : v}</div>
      </div>`;
    })
    .join("");
}

function pct(x) {
  return (x * 100).toFixed(1) + "%";
}

function fillSegTable(tbody, rows) {
  tbody.innerHTML = rows
    .map((r) => {
      const best = r.id === "s2_fullcover_qwen36";
      const m = r.match != null ? `${r.match}/${r.pred}/${r.gold}` : "—";
      const pr = r.p != null ? `${r.p.toFixed(3)} / ${r.r.toFixed(3)}` : "—";
      return `<tr class="${best ? "best" : ""}">
        <td>${r.name}</td>
        <td class="num">${r.f1.toFixed(4)}</td>
        <td class="num">${pr}</td>
        <td class="num">${m}</td>
        <td>${r.model}</td>
        <td>${r.full25 ? "25" : "smoke"}</td>
        <td>${r.note || ""}</td>
      </tr>`;
    })
    .join("");
}

function fillLabelTable(tbody, rows) {
  tbody.innerHTML = rows
    .map((r) => {
      const delta = ((r.acc - 0.506) * 100).toFixed(1);
      const cls = delta > 0 ? "delta-up" : delta < 0 ? "delta-down" : "";
      return `<tr class="${r.id === "l2_hawor" ? "best" : ""}">
        <td>${r.name}</td>
        <td class="num">${pct(r.acc)}</td>
        <td class="num">${r.n_match}/${r.n}</td>
        <td>${r.model}</td>
        <td class="num ${cls}">${delta > 0 ? "+" : ""}${delta}pp</td>
        <td>${r.note || ""}</td>
      </tr>`;
    })
    .join("");
}

function fillE2ETable(tbody, rows) {
  tbody.innerHTML = rows
    .map((r) => {
      const best = r.id === "selector397";
      return `<tr class="${best ? "best" : ""}">
        <td>${r.name}</td>
        <td class="num">${r.seg_f1.toFixed(4)}</td>
        <td class="num">${r.e2e_f1.toFixed(4)}</td>
        <td class="num">${r.pred_gold || "—"}</td>
        <td>${r.note || ""}</td>
      </tr>`;
    })
    .join("");
}

function renderTimeline(el, toy) {
  const dur = 10;
  const lane = (items, cls) =>
    items
      .map((s) => {
        const left = (s.start / dur) * 100;
        const width = ((s.end - s.start) / dur) * 100;
        return `<div class="seg ${cls}" style="left:${left}%;width:${width}%">${s.id}</div>`;
      })
      .join("");
  el.innerHTML = `
    <div class="lane"><div class="lane-title">Gold（3 段）</div><div class="lane-track">${lane(toy.gold, "gold")}</div></div>
    <div class="lane"><div class="lane-title">Pred after outer snap（4 段）</div><div class="lane-track">${lane(toy.after_snap, "pred")}</div></div>
    <p style="font-size:0.85rem;color:#5c564c;margin:0.6rem 0 0">
      IoU P0–G0=${toy.ious["P0-G0"]}, P1–G1=${toy.ious["P1-G1"]} ≥ 0.75 → n_match=${toy.n_match}；
      P=${toy.P}, R≈${toy.R}, F1≈${toy.F1}
    </p>`;
}

loadResults().then((data) => {
  fillSegTable(document.querySelector("#seg-tbody"), data.segmentation);
  fillLabelTable(document.querySelector("#label-tbody"), data.labeling);
  fillE2ETable(document.querySelector("#e2e-tbody"), data.e2e);
  barChart(document.querySelector("#seg-bars"), data.segmentation, "f1", 0.25);
  barChart(
    document.querySelector("#label-bars"),
    data.labeling.map((r) => ({ ...r, acc100: r.acc * 100 })),
    "acc100",
    70,
    true
  );
  barChart(document.querySelector("#e2e-bars"), data.e2e, "e2e_f1", 0.2);
  renderTimeline(document.querySelector("#toy-timeline"), data.walkthrough_toy);
}).catch((err) => {
  console.error(err);
  const el = document.querySelector("#load-error");
  if (el) el.hidden = false;
});
