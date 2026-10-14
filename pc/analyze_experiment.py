#!/usr/bin/env python3
"""Summarise a labelled box-experiment log from pc/gateway.py.

During the experiment each trial is started in the gateway with `t NAME`
(ARM + label). NAME = category + parameter + trial number, for example:

  drop5_1 .. drop20_5   drop height in cm
  tilt30_1 .. tilt60_3  tilt angle in degrees
  handle_1 .. handle_20 normal handling (pick up, carry, put down)
  vib_1 .. vib_3        vibration
  open_1 .. open_3      box opened (light)

Outputs, next to the CSV:
  <run>_trials.csv   one row per trial
  <run>_summary.md   per-condition tables for the report
  <run>_figure.png   drop height vs peak g, tilt angle vs alert level

Usage:
  python3 pc/analyze_experiment.py                  # pc/data/latest.csv
  python3 pc/analyze_experiment.py pc/data/run_X.csv
"""
import os
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

SURFACE, TEXT, TEXT_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
STATUS = {0: "#0ca30c", 1: "#fab219", 2: "#ec835a", 3: "#d03b3b"}
LEVEL_NAME = {0: "L0", 1: "L1", 2: "L2", 3: "L3"}
LEVEL_MARKER = {0: "o", 1: "^", 2: "D", 3: "s"}  # shape backs up colour
PROFILE_G_CRIT = {"GLASS": 4.0, "ELECTRONICS": 8.0, "ROBUST": 14.0}
NODE_ROWS = ["TELEMETRY", "EVENT", "ALERT", "PRESENCE"]


def per_trial(d: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for label, g in d[d["label"].fillna("") != ""].groupby("label", sort=False):
        m = re.match(r"([a-z]+)(\d*)", label)
        if not m:
            continue
        cat, param = m.group(1), m.group(2)
        start = g["pc_time"].iloc[0]
        node = g[g["type"].isin(NODE_ROWS)]
        ev = g[g["type"] == "EVENT"]
        top = ev.loc[ev["peak_g"].idxmax()] if not ev.empty else None
        l2 = node[node["level"] >= 2]
        rows.append({
            "label": label,
            "category": cat,
            "param": int(param) if param else None,
            "profile": node["profile"].dropna().iloc[-1] if not node.empty else "",
            "n_shock_events": len(ev),
            "peak_g": round(float(top["peak_g"]), 2) if top is not None else 0.0,
            "dv_mms": int(top["dv_mms"]) if top is not None else 0,
            "saturated": int(ev["sat"].fillna(0).max()) if not ev.empty else 0,
            "max_level": int(node["level"].max()) if not node.empty else 0,
            "max_tilt_deg": float(node["tilt_deg"].max()) if not node.empty else 0.0,
            "time_to_L2_s": round((l2["pc_time"].iloc[0] - start).total_seconds(), 1) if not l2.empty else None,
            "open_seen": int(node["open"].fillna(0).max()) if not node.empty else 0,
            "acked": int((g["type"] == "ACK_LOG").any()),
        })
    return pd.DataFrame(rows)


def summarise(t: pd.DataFrame) -> str:
    out = []
    drop = t[t["category"] == "drop"]
    if not drop.empty:
        out.append("### Drop test (box on padding)\n")
        out.append("| Height (cm) | Trials | Peak g, mean (min-max) | dV mm/s, mean | Saturated | Alert levels |")
        out.append("|---|---|---|---|---|---|")
        for h, g in drop.groupby("param"):
            lv = ", ".join(f"{LEVEL_NAME[k]}×{v}" for k, v in sorted(g["max_level"].value_counts().items()))
            out.append(f"| {int(h)} | {len(g)} | {g['peak_g'].mean():.1f} ({g['peak_g'].min():.1f}-{g['peak_g'].max():.1f}) "
                       f"| {g['dv_mms'].mean():.0f} | {int(g['saturated'].sum())} | {lv} |")
        out.append("")
    tilt = t[t["category"] == "tilt"]
    if not tilt.empty:
        out.append("### Tilt test\n")
        out.append("| Angle (deg) | Trials | Measured max tilt, mean | Alert levels | Time to L2 (s), mean |")
        out.append("|---|---|---|---|---|")
        for a, g in tilt.groupby("param"):
            lv = ", ".join(f"{LEVEL_NAME[k]}×{v}" for k, v in sorted(g["max_level"].value_counts().items()))
            ttl = g["time_to_L2_s"].dropna()
            out.append(f"| {int(a)} | {len(g)} | {g['max_tilt_deg'].mean():.1f} | {lv} | "
                       f"{ttl.mean():.1f} |" if not ttl.empty else
                       f"| {int(a)} | {len(g)} | {g['max_tilt_deg'].mean():.1f} | {lv} | - |")
        out.append("")
    for cat, title in (("handle", "Normal handling (false-alarm check)"), ("vib", "Vibration")):
        g = t[t["category"] == cat]
        if g.empty:
            continue
        false_alarms = int((g["max_level"] >= 2).sum())
        out.append(f"### {title}\n")
        out.append(f"- Trials: {len(g)}; trials reaching L2 or higher: **{false_alarms}** "
                   f"({100 * false_alarms / len(g):.0f}%)")
        out.append(f"- Shock events per trial: mean {g['n_shock_events'].mean():.1f}; "
                   f"highest peak {g['peak_g'].max():.1f} g\n")
    g = t[t["category"] == "open"]
    if not g.empty:
        out.append("### Box opening (light)\n")
        out.append(f"- Trials: {len(g)}; box-open flag seen: **{int(g['open_seen'].sum())}**\n")
    return "\n".join(out)


def style(ax, title):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", fontsize=10, color=TEXT)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.tick_params(colors=TEXT_2, labelsize=8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)


def figure(t: pd.DataFrame, path: str):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.2), facecolor=SURFACE)
    drop = t[t["category"] == "drop"]
    style(a1, "Drop height vs measured peak acceleration")
    for lvl in sorted(drop["max_level"].unique()):
        g = drop[drop["max_level"] == lvl]
        a1.scatter(g["param"], g["peak_g"], s=56, color=STATUS[lvl], marker=LEVEL_MARKER[lvl],
                   edgecolors=SURFACE, linewidths=2, label=f"reached {LEVEL_NAME[lvl]}", zorder=3)
    profile = drop["profile"].iloc[0] if not drop.empty else ""
    if profile in PROFILE_G_CRIT:
        a1.axhline(PROFILE_G_CRIT[profile], color=TEXT_2, linestyle="--", linewidth=1)
        a1.text(a1.get_xlim()[0], PROFILE_G_CRIT[profile], f" {profile} limit", fontsize=8,
                color=TEXT_2, va="bottom")
    a1.axhline(15, color=TEXT_2, linestyle=":", linewidth=1)
    a1.text(a1.get_xlim()[0], 15, " sensor saturation (~±16 g)", fontsize=8, color=TEXT_2, va="bottom")
    a1.set_xlabel("drop height (cm)", fontsize=8, color=TEXT_2)
    a1.set_ylabel("peak g", fontsize=8, color=TEXT_2)
    a1.set_ylim(0, 17)
    if not drop.empty:
        a1.set_xticks(sorted(drop["param"].dropna().unique()))
        a1.set_xlim(0, drop["param"].max() + 5)
    if not drop.empty:
        a1.legend(fontsize=8, frameon=False)

    tilt = t[t["category"] == "tilt"]
    style(a2, "Tilt angle vs alert level reached")
    for lvl in sorted(tilt["max_level"].unique()):
        g = tilt[tilt["max_level"] == lvl]
        a2.scatter(g["param"], g["max_level"], s=56, color=STATUS[lvl], marker=LEVEL_MARKER[lvl],
                   edgecolors=SURFACE, linewidths=2, zorder=3, label=f"{LEVEL_NAME[lvl]}")
    a2.set_yticks([0, 1, 2, 3], ["L0", "L1", "L2", "L3"])
    a2.set_ylim(-0.4, 3.4)
    a2.set_xlabel("held tilt angle (deg)", fontsize=8, color=TEXT_2)
    if not tilt.empty:
        a2.set_xticks(sorted(tilt["param"].dropna().unique()))
        a2.set_xlim(tilt["param"].min() - 10, tilt["param"].max() + 10)
    fig.tight_layout()
    fig.savefig(path, dpi=160, facecolor=SURFACE)


def main():
    default = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "latest.csv")
    path = os.path.realpath(sys.argv[1] if len(sys.argv) > 1 else default)
    d = pd.read_csv(path)
    if "label" not in d.columns or d["label"].fillna("").eq("").all():
        sys.exit("No labelled rows. Start each trial in the gateway with `t NAME` (e.g. t drop10_1).")
    d["pc_time"] = pd.to_datetime(d["pc_time"])
    t = per_trial(d)
    base = os.path.splitext(path)[0]
    t.to_csv(base + "_trials.csv", index=False)
    md = summarise(t)
    with open(base + "_summary.md", "w") as f:
        f.write(f"# Box experiment summary ({os.path.basename(path)})\n\n{md}\n")
    figure(t, base + "_figure.png")
    print(md)
    print(f"\nSaved {base}_trials.csv, {base}_summary.md, {base}_figure.png")


if __name__ == "__main__":
    main()
